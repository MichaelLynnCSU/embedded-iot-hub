/******************************************************************************
 * \file uart_transport.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-15
 *
 * \brief UART transport layer — BeagleBone <-> STM32 BlackPill.
 *
 * \details Manages full-duplex /dev/ttyS1:
 *
 *          TX (BeagleBone -> BlackPill):
 *            Sensor bundles pushed by uart_push_msg(). Called by
 *            uart_push_thread in uart_controller.c once per second.
 *            Writes serialised by g_uart_write_mutex.
 *
 *          RX (BlackPill -> BeagleBone):
 *            Heartbeat frames only. Format: "HB:1,<tx_id>\n".
 *            uart_reader_thread() reads line-framed input and calls
 *            uart_parse_line() on each complete line.
 *            uart_parse_line() calls heartbeat_stamp(DEV_LCD) to keep
 *            the LCD alive in the heartbeat monitor. Nothing else.
 *
 *          The BlackPill is a display consumer only. It does not send
 *          device state back to the BeagleBone. All sensor state flows
 *          inbound via the pipe from sensor_server and is authoritative
 *          in SHM and SQLite. The API reads from those sources directly.
 *
 *          tx_id is a uint16_t counter incremented by the BlackPill
 *          before each HB transmit. Gaps in tx_id in the log indicate
 *          missed HB frames.
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
#include "uart_controller.h"
#include "uart_transport.h"

#define UART_RETRY_DELAY_SEC    5       /**< delay before retrying UART open  */
#define UART_PUSH_DELAY_US      100000  /**< delay after write in microseconds */
#define UART_BAUD               B115200 /**< UART baud rate                   */

static int             g_uart_fd          = -1;
static pthread_mutex_t g_uart_write_mutex = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************************
 * \brief Push one UartFrame onto the ring buffer.
 *
 * \details Protected by g_uart_ring.mutex. When the ring is full the
 *          oldest frame is silently overwritten — newest data wins.
 *          Caller must call sem_post(&g_uart_frame_sem) after this returns.
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
 * \return 1 if a frame was popped, 0 if the ring was empty.
 *
 * \details Called only after sem_timedwait(&g_uart_frame_sem) succeeds.
 *          Empty check is a safety guard against semaphore/ring mismatch.
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
 * \return File descriptor on success, -1 on failure.
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

   LOG("UART opened: %s @ 115200 (full-duplex)", p_dev);
   return fd;
}

/******************************************************************************
 * \brief Parse one inbound UART line from the STM32 BlackPill.
 *
 * \param p_line - Null-terminated line string read from /dev/ttyS1.
 *
 * \details Only HB frames are expected from the BlackPill. Any other
 *          frame is logged and discarded.
 *
 *          HB:1,<tx_id>
 *            Stamps DEV_LCD in the heartbeat monitor. tx_id gaps in
 *            the log indicate missed frames.
 ******************************************************************************/
static void uart_parse_line(const char *p_line)
{
   char        buf[UART_LINE_LEN] = {0};
   char       *p_colon            = NULL;
   const char *p_id               = NULL;
   char       *p_rest             = NULL;

   (void)strncpy(buf, p_line, sizeof(buf) - 1);

   p_colon = strchr(buf, ':');
   if (NULL == p_colon)
   {
      LOG("[UART] Bad frame (no colon): %s", p_line);
      return;
   }

   *p_colon = '\0';
   p_id     = buf;
   p_rest   = p_colon + 1;

   if (0 == strcmp(p_id, "HB"))
   {
      char  tmp[UART_LINE_LEN] = {0};
      char *p_tok              = NULL;
      int   hb_val             = 0;
      int   tx_id              = 0;

      (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
      tmp[sizeof(tmp) - 1u] = '\0';

      p_tok = strtok(tmp, ",");
      if (NULL != p_tok) { hb_val = atoi(p_tok); }

      p_tok = strtok(NULL, ",");
      if (NULL != p_tok) { tx_id = atoi(p_tok); }

      LOG("[UART] transport=ttyS1 read src=blackpill_lcd id=HB val=%d tx_id=%d",
          hb_val, tx_id);
      heartbeat_stamp(DEV_LCD);
      return;
   }

   LOG("[UART] Unexpected frame from BlackPill (ignored): %s", p_line);
}

/******************************************************************************
 * \brief Thread — reads line-framed UART data from STM32 BlackPill.
 *
 * \param p_arg - Unused.
 *
 * \details Listens on /dev/ttyS1 for inbound HB frames. Reopens the
 *          device automatically on read error or EOF.
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
 * \details Serialises writes with g_uart_write_mutex. /dev/ttyS1 is
 *          full-duplex — the mutex protects concurrent writers only,
 *          not read/write concurrency (kernel handles that).
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
 ******************************************************************************/
int uart_transport_is_open(void)
{
   return (0 <= g_uart_fd) ? 1 : 0;
}
