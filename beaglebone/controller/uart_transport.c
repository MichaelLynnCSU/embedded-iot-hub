/******************************************************************************
 * \file uart_transport.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-15
 *
 * \brief UART transport layer for BeagleBone data controller.
 *
 * \details Extracted from uart_controller.c (2026-06-15 refactor: split
 *          into transport / protocol / correlation / lock-adapter modules,
 *          no behavior change). See uart_transport.h for design rationale.
 *
 *          This file owns:
 *            - g_uart_fd lifecycle (uart_open, uart_reader_thread)
 *            - line framing + per-line dispatch (uart_parse_line)
 *            - ring buffer push/pop (uart_ring_push/uart_ring_pop)
 *            - uart_push_msg() — the only UART write primitive
 *
 *          uart_parse_line() handles inbound frames from the STM32
 *          BlackPill on /dev/ttyS1. The STM32 is a display consumer —
 *          it receives sensor bundles from uart_push_thread() and sends
 *          back heartbeat (HB) frames and future command frames (LGT/LCK)
 *          triggered by user interaction on the LVGL display.
 *
 *          On receiving a frame, uart_parse_line():
 *            1. Stamps the device heartbeat
 *            2. Saves to DB
 *            3. Routes to the appropriate staging function
 *            4. Pushes onto g_uart_ring and signals g_uart_frame_sem
 *            5. Writes updated snapshot to SHM via handle_get_latest()
 *               with src=uart_ingress so the blackpill_lcd and
 *               thermostat_lcd readers see the update immediately
 *
 * \note    Counting semaphore + ring buffer (2026-05-22):
 *          uart_ring_push() and uart_ring_pop() — single-producer
 *          single-consumer ring buffer (capacity UART_RING_SIZE frames).
 *          uart_parse_line() calls uart_ring_push() then sem_post() so
 *          the semaphore count equals the number of frames in the buffer
 *          at all times. uart_push_thread() calls sem_timedwait() then
 *          uart_ring_pop() — the successful wait guarantees a frame
 *          exists, so pop never races.
 ******************************************************************************/

#include <termios.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include "config.h"
#include "sensor_types.h"
#include "log.h"
#include "globals.h"
#include "heartbeat.h"
#include "db_manager.h"
#include "uart_controller.h"
#include "uart_transport.h"
#include "uart_lock.h"
#include "cmd/uart_staging.h"
#include "cmd/state_registry.h"
#include "cmd/shm_updater.h"

#define UART_RETRY_DELAY_SEC    5       /**< delay before retrying UART open */
#define UART_PUSH_DELAY_US      100000  /**< delay after write in microseconds */
#define UART_BAUD               B115200 /**< UART baud rate */

static int             g_uart_fd          = -1;  /**< UART file descriptor */
static pthread_mutex_t g_uart_write_mutex =
   PTHREAD_MUTEX_INITIALIZER;                     /**< UART write mutex */

/******************************************************************************
 * \brief Push one UartFrame onto the ring buffer.
 *
 * \details Protected by g_uart_ring.mutex so multiple callers cannot
 *          corrupt head simultaneously (only uart_reader_thread calls
 *          this today, but the guard makes it safe if that changes).
 *
 *          When the ring is full the oldest frame is silently overwritten
 *          and a warning is logged. For sensor data, the newest reading
 *          is always more relevant than the oldest — last-write-wins is
 *          the correct overflow policy here.
 *
 *          Caller must call sem_post(&g_uart_frame_sem) after this
 *          returns to keep the semaphore count in sync with ring depth.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_ring_push(const UartFrame *p_frame)
{
   unsigned next_head = 0;

   pthread_mutex_lock(&g_uart_ring.mutex);

   next_head = (g_uart_ring.head + 1) & (UART_RING_SIZE - 1);

   if (next_head == g_uart_ring.tail)
   {
      LOG_WRN("[RING] Ring full — dropping oldest UART frame");
      g_uart_ring.tail = (g_uart_ring.tail + 1) & (UART_RING_SIZE - 1);
   }

   g_uart_ring.frames[g_uart_ring.head] = *p_frame;
   g_uart_ring.head                     = next_head;

   pthread_mutex_unlock(&g_uart_ring.mutex);
}

/******************************************************************************
 * \brief Pop one UartFrame from the ring buffer.
 *
 * \return int - 1 if a frame was popped, 0 if the ring was empty.
 *
 * \details Called only after sem_timedwait(&g_uart_frame_sem) succeeds,
 *          so the ring should never be empty at the call site. The empty
 *          check is a safety guard — if it fires it indicates a semaphore
 *          count / ring depth mismatch, which should never happen.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int uart_ring_pop(UartFrame *p_frame)
{
   if (g_uart_ring.tail == g_uart_ring.head)
   {
      LOG_WRN("[RING] Pop called on empty ring — semaphore mismatch");
      return 0;
   }

   *p_frame         = g_uart_ring.frames[g_uart_ring.tail];
   g_uart_ring.tail = (g_uart_ring.tail + 1) & (UART_RING_SIZE - 1);

   return 1;
}

/******************************************************************************
 * \brief Open UART device at 115200 8N1.
 *
 * \param p_dev - Device path string (e.g. /dev/ttyS1).
 *
 * \return int - File descriptor on success, -1 on failure.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static int uart_open(const char *p_dev)
{
   int            fd  = -1;
   struct termios tty;

   fd = open(p_dev, O_RDWR | O_NOCTTY | O_SYNC);
   if (0 > fd)
   {
      LOG("UART open failed: %s", p_dev);
      return -1;
   }

   (void)memset(&tty, 0, sizeof(tty));

   if (0 != tcgetattr(fd, &tty))
   {
      LOG("tcgetattr failed");
      close(fd);
      return -1;
   }

   (void)cfsetospeed(&tty, UART_BAUD);
   (void)cfsetispeed(&tty, UART_BAUD);

   tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
   tty.c_iflag &= ~IGNBRK;
   tty.c_lflag  = 0;
   tty.c_oflag  = 0;
   tty.c_cc[VMIN]  = 1;
   tty.c_cc[VTIME] = 0;
   tty.c_iflag &= ~(IXON | IXOFF | IXANY);
   tty.c_cflag |=  (CLOCAL | CREAD);
   tty.c_cflag &= ~(PARENB | PARODD);
   tty.c_cflag &= ~CSTOPB;
   tty.c_cflag &= ~CRTSCTS;

   if (0 != tcsetattr(fd, TCSANOW, &tty))
   {
      LOG("tcsetattr failed");
      close(fd);
      return -1;
   }

   LOG("UART opened: %s @ 115200", p_dev);
   return fd;
}

/******************************************************************************
 * \brief Parse and process one inbound UART frame from STM32 BlackPill.
 *
 * \param p_line - Null-terminated line string read from /dev/ttyS1.
 *
 * \return void
 *
 * \details Parses <ID>:<value>[,<batt>] format. Supported inbound IDs:
 *            HB  — heartbeat from BlackPill, stamps device liveness only
 *            LGT — light command (future: user button press on LVGL display)
 *            LCK — lock command  (future: user button press on LVGL display)
 *            PIR — PIR state     (future)
 *
 *          On a valid frame:
 *            1. heartbeat_stamp() — marks device online
 *            2. db_save_uart()    — persists to SQLite
 *            3. staging function  — updates state registry
 *            4. uart_ring_push() + sem_post() — wakes uart_push_thread
 *            5. handle_get_latest(src=uart_ingress) — projects updated
 *               snapshot to SHM so blackpill_lcd and thermostat_lcd
 *               readers see the change without waiting for the next
 *               pipe_ingress frame
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void uart_parse_line(const char *p_line)
{
   char        buf[UART_LINE_LEN] = {0};
   char       *p_colon  = NULL;
   char       *p_rest   = NULL;
   char       *p_comma  = NULL;
   const char *p_id     = NULL;
   int         val      = 0;
   int         batt     = -1;
   DEV_ID_E    idx      = (DEV_ID_E)-1;
   UartFrame   frame    = {0};
   struct LatestData snap;

   (void)strncpy(buf, p_line, sizeof(buf) - 1);

   p_colon = strchr(buf, ':');
   if (NULL == p_colon)
   {
      LOG("[UART] Bad frame: %s", p_line);
      return;
   }

   *p_colon = '\0';
   p_id     = buf;
   p_rest   = p_colon + 1;

   p_comma = strchr(p_rest, ',');
   if (NULL != p_comma)
   {
      *p_comma = '\0';
      batt     = atoi(p_comma + 1);
   }

   val = atoi(p_rest);

   if (0 == strcmp(p_id, "PIR"))
   {
      idx = DEV_PIR;
   }
   else if (0 == strcmp(p_id, "LGT"))
   {
      idx = DEV_LIGHT;
   }
   else if (0 == strcmp(p_id, "LCK"))
   {
      idx = DEV_LOCK;
   }
   else
   {
      LOG("[UART] Unknown ID: %s", p_id);
      return;
   }

   heartbeat_stamp(idx);
   LOG("[UART] %-5s val=%-3d batt=%d", p_id, val, batt);
   db_save_uart(p_id, val, batt);

   if (DEV_LOCK == idx)
   {
      uart_process_lock(val, batt);
   }
   else if (DEV_PIR == idx)
   {
      uart_stage_pir(val);
   }
   else if (DEV_LIGHT == idx)
   {
      uart_stage_light(val);
   }

   /* Push frame onto ring buffer then signal uart_push_thread.
    * Order is mandatory: push before post so the frame is visible
    * the moment uart_push_thread wakes from sem_timedwait.           */
   frame.idx  = idx;
   frame.val  = val;
   frame.batt = batt;
   uart_ring_push(&frame);
   sem_post(&g_uart_frame_sem);

   /* Project updated snapshot to SHM immediately so blackpill_lcd and
    * thermostat_lcd readers see the change without waiting for the
    * next pipe_ingress frame.                                         */
   get_snapshot(&snap);
   handle_get_latest(&snap, "uart_ingress");
}

/******************************************************************************
 * \brief Thread — reads line-framed UART data from STM32 BlackPill.
 *
 * \param p_arg - Unused thread argument.
 *
 * \return void*
 *
 * \details Listens on /dev/ttyS1 for inbound frames from the STM32
 *          BlackPill — heartbeat and future command frames. Owns
 *          g_uart_fd lifecycle. Reopens the device automatically on
 *          read error or EOF. Character-by-character accumulation into
 *          line[] with overflow protection.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void *uart_reader_thread(void *p_arg)
{
   char    line[UART_LINE_LEN] = {0};
   int     pos                 = 0;
   char    c                   = 0;
   ssize_t n                   = 0;

   (void)p_arg;

   while (running)
   {
      if (0 > g_uart_fd)
      {
         g_uart_fd = uart_open(UART_DEV);
         if (0 > g_uart_fd)
         {
            sleep(UART_RETRY_DELAY_SEC);
            continue;
         }
      }

      while (running)
      {
         n = read(g_uart_fd, &c, 1);

         if (0 > n)
         {
            LOG("[UART] Read error");
            close(g_uart_fd);
            g_uart_fd = -1;
            break;
         }

         if (0 == n) { continue; }

         if (('\n' == c) || ('\r' == c))
         {
            if (0 < pos)
            {
               line[pos] = '\0';
               uart_parse_line(line);
               pos = 0;
            }
         }
         else if (pos < (UART_LINE_LEN - 1))
         {
            line[pos] = c;
            pos++;
         }
         else
         {
            LOG("[UART] Line overflow");
            pos = 0;
         }
      }
   }

   if (0 <= g_uart_fd)
   {
      close(g_uart_fd);
      g_uart_fd = -1;
   }

   LOG("[UART] Reader thread exiting");
   return NULL;
}

/******************************************************************************
 * \brief Write a message to UART with mutex protection.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_push_msg(const UartMsg *p_msg)
{
   ssize_t w = 0;

   LOG("[PUSH] bundle len=%d:\n%.*s", p_msg->len, p_msg->len, p_msg->buf);

   pthread_mutex_lock(&g_uart_write_mutex);
   w = write(g_uart_fd, p_msg->buf, p_msg->len);
   pthread_mutex_unlock(&g_uart_write_mutex);

   usleep(UART_PUSH_DELAY_US);

   if (w == (ssize_t)p_msg->len)
   {
      LOG("[PUSH] Sent: %.*s", p_msg->len - 1, p_msg->buf);
   }
   else
   {
      LOG("[PUSH] Write failed (w=%zd)", w);
   }
}

/******************************************************************************
 * \brief Return 1 if the UART device is currently open, 0 otherwise.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int uart_transport_is_open(void)
{
   return (0 <= g_uart_fd) ? 1 : 0;
}
