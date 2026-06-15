/******************************************************************************
 * \file uart_controller.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief UART controller for BeagleBone data controller.
 *
 * \details Manages bidirectional UART communication with the STM32
 *          blue pill. Two threads run concurrently:
 *
 *          uart_reader_thread — reads line-framed data from STM32,
 *          parses device frames, stamps heartbeats, saves to DB,
 *          pushes parsed frames onto g_uart_ring, and signals
 *          uart_push_thread via g_uart_frame_sem.
 *
 *          uart_push_thread — blocks on g_uart_frame_sem rather than
 *          sleeping on a fixed timer. Wakes immediately when a frame
 *          arrives, pops from g_uart_ring, builds and sends the
 *          consolidated state to STM32. Falls back to a 5s deadline
 *          (UART_PUSH_INTERVAL_SEC) as a watchdog push if no frames
 *          arrive — preserving the existing heartbeat behavior.
 *
 *          uart_update_frame — synchronous outbound push called from
 *          sensor_frame_dispatch() on every TCP ingress frame. Builds
 *          the full STM32 protocol payload directly from the frozen
 *          snapshot. Replaces the async semaphore-wake model for the
 *          TCP ingress path. uart_push_thread remains for UART-ingress
 *          frames (PIR/LGT/LCK) which do not go through
 *          sensor_frame_dispatch().
 *
 *          UART frame format (inbound from STM32):
 *          <ID>:<value>[,<batt>]\n
 *          Valid IDs: PIR, LGT, LCK
 *
 *          Push protocol (outbound to STM32):
 *          STATE:tmp,pir,lgt,lck,age_pir,age_lgt,age_lck,reed_count\n
 *          PIR:count[,batt]\n
 *          PIR_COUNT:n\n
 *          PIR<n>:count,batt,age\n  (one per active PIR slot)
 *          OCC<n>:occupied\n        (one per active PIR slot)
 *          REED_COUNT:n\n
 *          DR<n>:state,batt,age\n   (one per active reed slot)
 *          LGT:state\n
 *          LCK:state[,batt]\n
 *          MTR:online[,batt]\n
 *          TEMP_COUNT:n\n
 *          TEMP<n>:decidegc,batt,age\n (one per active temp slot)
 *          DB<n>:age_s,online\n     (one per doorbell cam slot, 0-indexed)
 *          DOORBELL:pressed,device_id[,person,conf_pct,asset]\n
 *          CAM<n>:online\n          (one per inference cam, 1-indexed)
 *
 *          DOORBELL extended fields (person, conf_pct, asset) are only
 *          emitted when doorbell_pressed is non-zero AND
 *          doorbell_result_reader_poll() returns a new inference result
 *          from the /doorbell_result shm segment (written by
 *          doorbell_daemon). When no inference result is available for
 *          a press event, the two-field form is sent instead so the STM32
 *          always sees at least pressed/device_id. asset is the
 *          "%Y%m%dT%H%M%SZ" timestamp string (17 chars), NOT the full
 *          saved filename — the full path is a storage-layer concern.
 *
 *          Lock state machine (LOCK_STATE_E from controller_logic.h):
 *          Inbound LCK frames drive logic_lock_transition(). Commands
 *          are rejected while motor is moving (UNLOCKING/LOCKING).
 *          All transitions are logged to device_events via db_save_event().
 *
 *          Adding ReedSensor3..N requires zero code changes here —
 *          ESP32 auto-discovers, reed_count increments in JSON,
 *          STM32 receives REED_COUNT:n and calls UI_Reflow(n).
 *
 * \note    PIR slot push (2026-05-XX):
 *          snapshot_pir_slots() added — mirrors snapshot_reed_slots().
 *          build_and_push() extended with PIR_COUNT:n and PIR<n>:count,batt,age
 *          lines so STM32 UI can render per-sensor PIR tiles.
 *
 * \note    Per-slot OCC frames (2026-05-20):
 *          Single OCC:n global frame replaced by OCC<n>:n per-slot frames.
 *          snapshot_pir_slots() gains p_occ[] array from
 *          pir_slots[i].occupied. build_and_push() emits
 *          OCC1:n..OCC<pir_count>:n after the PIR<n> lines.
 *          STM32 parser.c handles OCC<1-4> mirroring PIR<1-4> pattern.
 *
 * \note    Counting semaphore + ring buffer (2026-05-22):
 *          uart_ring_push() and uart_ring_pop() added — single-producer
 *          single-consumer ring buffer (capacity UART_RING_SIZE frames).
 *          uart_parse_line() calls uart_ring_push() then sem_post() so
 *          the semaphore count equals the number of frames in the buffer
 *          at all times. uart_push_thread() calls sem_timedwait() then
 *          uart_ring_pop() — the successful wait guarantees a frame exists,
 *          so pop never races. This eliminates payload loss from the previous
 *          latest_data overwrite pattern and removes the fixed sleep() poll.
 *          shm_sem replaced by shm_data->shm_mutex throughout.
 *
 * \note    Doorbell fix (2026-06-07):
 *          doorbell_pressed and doorbell_device_id are fields of shm_data
 *          (shared memory), not latest_data. uart_push_thread() and
 *          uart_update_frame() both read doorbell state from shm_data under
 *          shm_mutex and clear doorbell_pressed after reading (one-shot).
 *          Previously they were read from latest_data which is never
 *          populated with doorbell state, causing DOORBELL:0,x to be sent
 *          every push cycle.
 *
 * \note    State registry migration (2026-06-09):
 *          latest_data and data_mutex are now private to state_registry.c.
 *          All direct accesses in this file have been removed:
 *
 *          uart_process_lock() — previously wrote latest_data.lock_state
 *          and latest_data.batt_lck under data_mutex. Now calls
 *          uart_stage_lock() which delegates to state_registry internally.
 *          g_lock_mutex (private to this file) guards g_lock_state.
 *          The two mutexes are never held simultaneously — no ABBA risk.
 *
 *          uart_parse_line() — previously wrote latest_data.motion_count,
 *          latest_data.valid, and latest_data.light_state under data_mutex.
 *          Now calls uart_stage_pir() / uart_stage_light() which delegate
 *          to state_registry internally.
 *
 *          snapshot_reed_slots(), snapshot_pir_slots(), snapshot_temp_slots()
 *          — all three deleted. uart_push_thread() and uart_parse_line() now
 *          call get_snapshot() once to obtain an atomic struct copy, then
 *          read directly from the snapshot local. All previous data_mutex
 *          lock/unlock pairs in this file are gone.
 *
 *          uart_sync_lock_state() — previously locked data_mutex to write
 *          g_lock_state. Now locks g_lock_mutex instead, which is the
 *          correct owner for g_lock_state after the registry split.
 *
 *          uart_update_frame() — synchronous outbound push. Builds full
 *          STM32 protocol payload from frozen snapshot via get_snapshot().
 *          uart_write_string/uart_printf removed — uart_push_msg() is the
 *          only send primitive in this file.
 *
 *          Net result: zero references to latest_data or data_mutex remain
 *          in this translation unit.
 *
 * \note    Inference tie-in — doorbell_result_reader (2026-06-14):
 *          doorbell_daemon (beaglebone/inference/app/) runs inference on
 *          every JPEG received over Lane B (TCP port 9091) and publishes
 *          results to the /doorbell_result POSIX shm segment
 *          (doorbell_result_shm.h: struct DoorbellResult — event_id,
 *          device_id, person, conf_pct, asset[20]).
 *
 *          doorbell_result_reader.c (accessor module, cmd/ pattern) wraps
 *          the shm attach and lock-free event_id change-detection:
 *            doorbell_result_reader_init()  — attach, lazy-retry-safe;
 *            doorbell_result_reader_poll()  — returns 1 + copies out
 *              person/conf_pct/asset if event_id has changed since last
 *              call, 0 otherwise. Retries attach internally on each call
 *              until doorbell_daemon creates the segment.
 *
 *          Integration points in this file:
 *
 *          uart_update_frame() (TCP ingress path) and uart_push_thread()
 *          (UART ingress path) — both call doorbell_pending_check() at the
 *          top of each push cycle. If a result is ready
 *          (g_db_pending_result_ready), doorbell_inject_pending() injects
 *          it into the current bundle with pressed=1 so the five-field
 *          DOORBELL frame goes out inside a complete well-formed payload —
 *          never as a standalone transmission (which collides on UART).
 *
 *          asset field: the 17-char "%Y%m%dT%H%M%SZ" timestamp string
 *          embedded in the full saved filename (not the filename itself).
 *          event_id is the authoritative identity for server-side lookup.
 *
 *          No other files in this translation unit require changes.
 *          doorbell_result_reader_init() need not be called explicitly at
 *          startup — poll() lazy-retries attach on every call.
 *
 * \note    Doorbell pending slot redesign (2026-06-15):
 *          doorbell_pending_check_and_push() replaced by two functions:
 *            doorbell_pending_check()  — polls shm, stores result in
 *              g_db_pending_result_ready + five result fields. Never calls
 *              uart_push_msg() directly.
 *            doorbell_inject_pending() — called by both push paths before
 *              their DOORBELL snprintf block. If result is ready and no new
 *              press this cycle, overwrites local doorbell vars with the
 *              stored result and sets pressed=1 so the five-field frame
 *              goes out inside the normal bundle.
 *          The old standalone push approach sent a separate uart_push_msg()
 *          which collided with the normal bundle on UART and was lost by
 *          the STM32 parser.
 ******************************************************************************/

#include <termios.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <stdlib.h>
#include "config.h"
#include "sensor_types.h"
#include "shared_data.h"
#include "log.h"
#include "globals.h"
#include "heartbeat.h"
#include "db_manager.h"
#include "uart_controller.h"
#include "cmd/shm_updater.h"
#include "controller_logic.h"
#include "cmd/uart_staging.h"
#include "cmd/state_registry.h"
#include "doorbell_result_reader.h"
#include "doorbell_result_shm.h"

#define DOORBELL_ASSET_LEN  20   /* matches DoorbellResult.asset[20] in doorbell_result_shm.h */

#define UART_PUSH_INTERVAL_SEC  5       /**< max wait in sem_timedwait — watchdog ceiling */
#define UART_RETRY_DELAY_SEC    5       /**< delay before retrying UART open */
#define UART_PUSH_DELAY_US      100000  /**< delay after write in microseconds */
#define UART_MSG_BUF_SIZE       1024    /**< push message buffer size bytes */
#define UART_STATE_UNKNOWN      0xFF    /**< sentinel for unknown reed state */
#define UART_BAUD               B115200 /**< UART baud rate */

static int             g_uart_fd          = -1;  /**< UART file descriptor */
static pthread_mutex_t g_uart_write_mutex =
   PTHREAD_MUTEX_INITIALIZER;                     /**< UART write mutex */

/**
 * \brief Lock state machine — owned here, protected by g_lock_mutex.
 *        state_registry owns all other sensor state. g_lock_state is
 *        separate because it drives the lock transition logic before
 *        the resolved state is written back to the registry.
 */
static LOCK_STATE_E    g_lock_state = LOCK_STATE_LOCKED;
static pthread_mutex_t g_lock_mutex = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************************
 * Doorbell inference "pending result" state (2026-06-15)
 *
 * Lane A (UDP press event -> shm_data->doorbell_pressed) arrives almost
 * immediately, but Lane B (TCP JPEG -> doorbell_daemon -> inference ->
 * /doorbell_result publish) can take several seconds. The original code
 * polled doorbell_result_reader_poll() exactly once, at the same moment
 * doorbell_pressed was read and cleared (one-shot) — so the poll almost
 * always missed and the five-field result was never delivered.
 *
 * Fix: when a press is seen but no result is available yet, remember it
 * here (single pending slot). Every subsequent push cycle calls
 * doorbell_pending_check() which polls the result shm. When the result
 * arrives it is stored in g_db_pending_result_ready and the five result
 * fields. doorbell_inject_pending() is called by both push paths before
 * their DOORBELL snprintf block — if a result is ready and no new press
 * arrived this cycle, it overwrites the local doorbell vars with pressed=1
 * so the five-field frame goes out inside the normal complete bundle.
 *
 * Sending the result as a standalone uart_push_msg() (previous approach)
 * collided with the normal push bundle on UART and was lost by the STM32
 * parser — UART is a single byte stream, two concurrent writes interleave.
 *
 * If no result arrives within DOORBELL_PENDING_TIMEOUT_SEC the pending
 * slot is cleared and a warning is logged. STM32 keeps the two-field
 * fallback from the original press cycle.
 *
 * g_db_pending_mutex guards all fields below since uart_push_thread and
 * the TCP sync path run on different threads and both call these functions.
 ******************************************************************************/
#define DOORBELL_PENDING_TIMEOUT_SEC 10

static pthread_mutex_t g_db_pending_mutex = PTHREAD_MUTEX_INITIALIZER;
static int             g_db_pending              = 0; /* 1 = press seen, result not yet consumed */
static int             g_db_pending_device       = 0;
static time_t          g_db_pending_since        = 0;
/* Result fields — populated by doorbell_pending_check(), injected by next push cycle */
static int             g_db_pending_result_ready = 0;
static uint64_t        g_db_pending_event_id     = 0;
static uint8_t         g_db_pending_person       = 0;
static uint8_t         g_db_pending_conf_pct     = 0;
static char            g_db_pending_asset[DOORBELL_ASSET_LEN];

/******************************************************************************
 * \brief Push one UartFrame onto the ring buffer.
 *
 * \param p_frame - Pointer to frame to copy into the ring.
 *
 * \return void
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
 * \param p_frame - Output pointer to receive the popped frame.
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
 * \brief Process an inbound LCK frame through the lock state machine.
 *
 * \param val  - Parsed lock value from inbound UART frame.
 * \param batt - Battery SOC percent, or -1 if absent.
 *
 * \return void
 *
 * \details g_lock_state is guarded by g_lock_mutex (private to this file).
 *          After resolving the new lock state the result is written into
 *          the registry via uart_stage_lock(), which acquires state_mutex
 *          internally. The two mutexes are never held simultaneously,
 *          eliminating any ABBA deadlock risk.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void uart_process_lock(int val, int batt)
{
   LOCK_STATE_E    old_state = LOCK_STATE_LOCKED;
   LOCK_STATE_E    new_state = LOCK_STATE_LOCKED;
   const char     *p_ev      = NULL;

   pthread_mutex_lock(&g_lock_mutex);

   old_state = g_lock_state;
   new_state = logic_lock_transition(old_state, val);

   if (new_state == old_state)
   {
      if (logic_lock_is_busy(old_state))
      {
         LOG("[LCK] Command rejected — motor moving (%s)",
             logic_lock_state_label(old_state));
      }
      pthread_mutex_unlock(&g_lock_mutex);
      return;
   }

   g_lock_state = new_state;
   pthread_mutex_unlock(&g_lock_mutex);

   /* Write resolved lock state into registry.
    * uart_stage_lock() acquires state_mutex internally — must be called
    * after g_lock_mutex is released to avoid holding both simultaneously. */
   uart_stage_lock((int)new_state, batt);

   p_ev = logic_lock_event_str(old_state, new_state);

   LOG("[LCK] %s -> %s (val=%d batt=%d)",
       logic_lock_state_label(old_state),
       logic_lock_state_label(new_state),
       val, batt);

   if (NULL != p_ev)
   {
      db_save_event("LCK", p_ev);
   }
}

/******************************************************************************
 * \brief Parse and process one inbound UART frame from STM32.
 *
 * \param p_line - Null-terminated line string read from UART device.
 *
 * \return void
 *
 * \details Parses <ID>:<value>[,<batt>] format. Stamps heartbeat, saves
 *          to DB, routes to the appropriate staging function, pushes onto
 *          g_uart_ring, signals g_uart_frame_sem, then pushes the updated
 *          snapshot to shm via handle_get_latest().
 *
 *          All registry writes go through uart_stage_pir(),
 *          uart_stage_light(), or uart_stage_lock() — no direct
 *          state_registry calls here.
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

   /* Push updated snapshot to shm immediately after staging so the
    * LCD display sees UART-ingress updates without waiting for the
    * next TCP frame cycle.                                           */
   get_snapshot(&snap);
   handle_get_latest(&snap);
}

/******************************************************************************
 * \brief Thread — reads line-framed UART data from STM32.
 *
 * \param p_arg - Unused thread argument.
 *
 * \return void*
 *
 * \details Owns g_uart_fd lifecycle. Reopens the device automatically
 *          on read error or EOF. Character-by-character accumulation
 *          into line[] with overflow protection.
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
 * \param p_msg - Pointer to message buffer.
 * \param len   - Length in bytes to write.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void uart_push_msg(const char *p_msg, int len)
{
   ssize_t w = 0;

   pthread_mutex_lock(&g_uart_write_mutex);
   w = write(g_uart_fd, p_msg, len);
   pthread_mutex_unlock(&g_uart_write_mutex);

   usleep(UART_PUSH_DELAY_US);

   if (w == (ssize_t)len)
   {
      LOG("[PUSH] Sent: %.*s", len - 1, p_msg);
   }
   else
   {
      LOG("[PUSH] Write failed (w=%zd)", w);
   }
}

/******************************************************************************
 * \brief Poll for late-arriving doorbell inference result and store it.
 *
 * \details If g_db_pending is set and g_db_pending_result_ready is not yet
 *          set, polls doorbell_result_reader_poll(). On a hit, stores the
 *          result fields in the pending globals and sets
 *          g_db_pending_result_ready = 1. The caller's next
 *          doorbell_inject_pending() call will consume these fields and
 *          inject them into the normal push bundle — never sent standalone.
 *
 *          On timeout, clears the pending slot and logs a warning.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void doorbell_pending_check(void)
{
   int      pending    = 0;
   int      device     = 0;
   time_t   since      = 0;
   time_t   now        = time(NULL);
   uint64_t event_id   = 0;
   uint8_t  person     = 0;
   uint8_t  conf_pct   = 0;
   char     asset[DOORBELL_ASSET_LEN] = {0};
   int      got_result = 0;

   pthread_mutex_lock(&g_db_pending_mutex);
   pending = g_db_pending;
   device  = g_db_pending_device;
   since   = g_db_pending_since;
   if (g_db_pending_result_ready)
   {
      /* Result already stored — wait for push cycle to consume it */
      pthread_mutex_unlock(&g_db_pending_mutex);
      return;
   }
   pthread_mutex_unlock(&g_db_pending_mutex);

   if (!pending) { return; }

   got_result = doorbell_result_reader_poll(&event_id, &person, &conf_pct,
                                             asset, sizeof(asset));
   if (got_result)
   {
      LOG("[CONTROLLER] result_ready event_id=" EVENT_ID_FMT
          " device_id=%d person=%d conf=%d",
          EVENT_ID_ARG(event_id), device, person, conf_pct);

      pthread_mutex_lock(&g_db_pending_mutex);
      g_db_pending_result_ready = 1;
      g_db_pending_event_id     = event_id;
      g_db_pending_person       = person;
      g_db_pending_conf_pct     = conf_pct;
      (void)strncpy(g_db_pending_asset, asset, DOORBELL_ASSET_LEN - 1);
      g_db_pending_asset[DOORBELL_ASSET_LEN - 1] = '\0';
      pthread_mutex_unlock(&g_db_pending_mutex);
      return;
   }

   if ((now - since) > DOORBELL_PENDING_TIMEOUT_SEC)
   {
      LOG("[CONTROLLER] result_timeout device_id=%d", device);
      pthread_mutex_lock(&g_db_pending_mutex);
      g_db_pending              = 0;
      g_db_pending_result_ready = 0;
      pthread_mutex_unlock(&g_db_pending_mutex);
   }
}

/******************************************************************************
 * \brief Inject a stored pending inference result into the current push cycle.
 *
 * \details If g_db_pending_result_ready is set and doorbell_pressed is
 *          currently 0 (no new press this cycle), overwrites the caller's
 *          local doorbell variables with the stored result fields and sets
 *          *p_pressed = 1 so the five-field DOORBELL frame goes out in the
 *          normal bundle. Clears the pending slot atomically under mutex.
 *
 *          Must be called after the shm doorbell read block and before the
 *          DOORBELL snprintf block in each push path.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void doorbell_inject_pending(int     *p_pressed,
                                    int     *p_device_id,
                                    int     *p_valid,
                                    uint64_t *p_event_id,
                                    uint8_t *p_person,
                                    uint8_t *p_conf_pct,
                                    char    *p_asset,
                                    int      asset_len)
{
   pthread_mutex_lock(&g_db_pending_mutex);

   if (!(*p_pressed) && g_db_pending_result_ready)
   {
      *p_pressed   = 1;
      *p_device_id = g_db_pending_device;
      *p_valid     = 1;
      *p_event_id  = g_db_pending_event_id;
      *p_person    = g_db_pending_person;
      *p_conf_pct  = g_db_pending_conf_pct;
      (void)strncpy(p_asset, g_db_pending_asset, asset_len - 1);
      p_asset[asset_len - 1]    = '\0';
      g_db_pending              = 0;
      g_db_pending_result_ready = 0;

      LOG("[CONTROLLER] inject_pending device_id=%d person=%d conf=%d",
          *p_device_id, *p_person, *p_conf_pct);
   }

   pthread_mutex_unlock(&g_db_pending_mutex);
}

/******************************************************************************
 * \brief Build UART push payload from explicit slot arrays and send to STM32.
 *
 * \details Called by uart_push_thread() only. Takes pre-extracted slot
 *          arrays rather than a snapshot struct so the thread can apply
 *          its own doorbell read/clear logic before calling here.
 *
 *          doorbell_pressed / doorbell_device_id are the shm-sourced press
 *          event fields (one-shot, cleared by caller after reading), or
 *          injected from the pending slot by doorbell_inject_pending().
 *          db_person / db_conf_pct / db_asset are the inference results —
 *          valid only when db_inference_valid is non-zero. When valid, the
 *          extended five-field DOORBELL frame is emitted; otherwise the
 *          two-field fallback is used.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void build_and_push(double temp, int motion, int lgt, int lck,
                            uint16_t age_pir, uint16_t age_lgt,
                            uint16_t age_lck, int8_t batt_pir,
                            int8_t batt_lck, int batt_motor,
                            int reed_count, int motor_online,
                            const uint8_t  *p_r_state,
                            const int8_t   *p_r_batt,
                            const uint16_t *p_r_age,
                            int             pir_count,
                            const uint32_t *p_pc,
                            const int8_t   *p_pb,
                            const uint16_t *p_pa,
                            const uint8_t  *p_pactive,
                            const int      *p_pocc,
                            int             temp_count,
                            const int16_t  *p_tc,
                            const int8_t   *p_tb,
                            const uint16_t *p_ta,
                            const uint8_t  *p_tactive,
                            int             doorbell_pressed,
                            int             doorbell_device_id,
                            int             db_inference_valid,
                            uint64_t        db_event_id,
                            uint8_t         db_person,
                            uint8_t         db_conf_pct,
                            const char     *db_asset)
{
   char msg[UART_MSG_BUF_SIZE] = {0};
   int  pos                    = 0;
   int  i                      = 0;

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "STATE:%d,%d,%d,%d,%d,%d,%d,%d\n",
                   (int)temp, motion, lgt, lck,
                   age_pir, age_lgt, age_lck, reed_count);

   if (0 <= batt_pir)
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "PIR:%d,%d\n", motion, batt_pir);
   }
   else
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "PIR:%d\n", motion);
   }

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "PIR_COUNT:%d\n", pir_count);

   for (i = 0; i < pir_count; i++)
   {
      if (!p_pactive[i]) { continue; }
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "PIR%d:%u,%d,%d\n",
                      i + 1, (unsigned)p_pc[i], p_pb[i], p_pa[i]);
   }

   for (i = 0; i < pir_count; i++)
   {
      if (!p_pactive[i]) { continue; }
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "OCC%d:%d\n", i + 1, p_pocc[i]);
   }

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "REED_COUNT:%d\n", reed_count);

   for (i = 0; i < reed_count; i++)
   {
      if (UART_STATE_UNKNOWN == p_r_state[i]) { continue; }
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "DR%d:%d,%d,%d\n",
                      i + 1, p_r_state[i], p_r_batt[i], p_r_age[i]);
   }

   pos += snprintf(msg + pos, sizeof(msg) - pos, "LGT:%d\n", lgt);

   if (0 <= batt_lck)
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "LCK:%d,%d\n", lck, batt_lck);
   }
   else
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos, "LCK:%d\n", lck);
   }

   if (batt_motor > 0)
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "MTR:%d,%d\n", motor_online, batt_motor);
   }
   else
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "MTR:%d\n", motor_online);
   }

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "TEMP_COUNT:%d\n", temp_count);

   for (i = 0; i < temp_count; i++)
   {
      if (!p_tactive[i]) { continue; }
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "TEMP%d:%d,%d,%d\n",
                      i + 1, p_tc[i], p_tb[i], p_ta[i]);
   }

   /* Emit extended DOORBELL frame when inference result is available,
    * two-field fallback otherwise — STM32 parser handles both forms. */
   if (doorbell_pressed && db_inference_valid)
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "DOORBELL:%d,%d,%d,%d,%s\n",
                      doorbell_pressed, doorbell_device_id,
                      db_person, db_conf_pct, db_asset);

      LOG("[CONTROLLER] -> [UART] send event_id=" EVENT_ID_FMT
          " device_id=%d person=%d conf=%d",
          EVENT_ID_ARG(db_event_id), doorbell_device_id,
          db_person, db_conf_pct);
   }
   else
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "DOORBELL:%d,%d\n",
                      doorbell_pressed, doorbell_device_id);
   }

   uart_push_msg(msg, pos);
}

/******************************************************************************
 * \brief Thread — sends sensor state to STM32, driven by g_uart_frame_sem.
 *
 * \param p_arg - Unused thread argument.
 *
 * \return void*
 *
 * \details Blocks on g_uart_frame_sem with UART_PUSH_INTERVAL_SEC watchdog
 *          ceiling. Wakes immediately when uart_parse_line() posts a frame,
 *          drains all pending frames from g_uart_ring, then builds and sends
 *          the full STM32 protocol payload from a single get_snapshot() call.
 *
 *          Handles UART-ingress frames (PIR/LGT/LCK) only. TCP-ingress
 *          frames are handled synchronously by uart_update_frame() called
 *          from sensor_frame_dispatch().
 *
 *          Doorbell state is read from shm_data under shm_mutex and cleared
 *          one-shot. doorbell_pending_check() polls for a late-arriving
 *          result; doorbell_inject_pending() injects it into the bundle if
 *          ready, so the five-field frame always goes out inside the normal
 *          complete payload — never as a standalone transmission.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void *uart_push_thread(void *p_arg)
{
   struct LatestData snap        = {0};

   int      valid              = 0;
   double   temp               = 0.0;
   int      motion             = 0;
   int      lgt                = 0;
   int      lck                = 0;
   uint16_t age_pir            = 0;
   uint16_t age_lgt            = 0;
   uint16_t age_lck            = 0;
   int8_t   batt_pir           = -1;
   int8_t   batt_lck           = -1;
   int      batt_motor         = -1;
   int      motor_online       = 0;
   int      reed_count         = 0;
   int      pir_count          = 0;
   int      temp_count         = 0;
   int      frames_ready       = 0;
   int      doorbell_pressed   = 0;
   int      doorbell_device_id = 0;
   int      db_inference_valid = 0;
   uint64_t db_event_id        = 0;
   uint8_t  db_person          = 0;
   uint8_t  db_conf_pct        = 0;
   char     db_asset[DOORBELL_ASSET_LEN];
   int      i                  = 0;
   UartFrame frame             = {0};

   uint8_t  r_state[MAX_REEDS];
   int8_t   r_batt[MAX_REEDS];
   uint16_t r_age[MAX_REEDS];

   uint32_t p_count[MAX_PIRS];
   int8_t   p_batt[MAX_PIRS];
   uint16_t p_age[MAX_PIRS];
   uint8_t  p_active[MAX_PIRS];
   int      p_occ[MAX_PIRS];

   int16_t  t_decidegc[MAX_TEMPS];
   int8_t   t_batt[MAX_TEMPS];
   uint16_t t_age[MAX_TEMPS];
   uint8_t  t_active[MAX_TEMPS];

   (void)p_arg;

   LOG("[PUSH] Push thread started (sem-driven, ceiling=%ds)",
       UART_PUSH_INTERVAL_SEC);

   while (running)
   {
      {
         struct timespec deadline;
         clock_gettime(CLOCK_REALTIME, &deadline);
         deadline.tv_sec += UART_PUSH_INTERVAL_SEC;
         sem_timedwait(&g_uart_frame_sem, &deadline);
      }

      if (!running) { break; }

      if (0 > g_uart_fd) { continue; }

      frames_ready = 0;
      if (uart_ring_pop(&frame)) { frames_ready = 1; }
      while (sem_trywait(&g_uart_frame_sem) == 0)
      {
         uart_ring_pop(&frame);
         frames_ready++;
      }

      LOG("[PUSH] Woke: %d UART frame(s) in burst", frames_ready);

      pthread_mutex_lock(&shm_data->shm_mutex);
      valid               = shm_data->data_valid;
      temp                = shm_data->current_temp;
      motion              = shm_data->current_motion;
      lgt                 = shm_data->current_light;
      lck                 = shm_data->current_lock;
      doorbell_pressed    = shm_data->doorbell_pressed;
      doorbell_device_id  = shm_data->doorbell_device_id;
      if (0 != doorbell_pressed) { shm_data->doorbell_pressed = 0; }
      pthread_mutex_unlock(&shm_data->shm_mutex);

      if (!valid)
      {
         LOG("[PUSH] No valid data yet, skipping");
         continue;
      }

      db_inference_valid = 0;
      db_event_id        = 0;
      db_person          = 0;
      db_conf_pct        = 0;
      db_asset[0]        = '\0';

      /* Check for late-arriving result from a previous press cycle */
      doorbell_pending_check();

      if (0 != doorbell_pressed)
      {
         db_inference_valid = doorbell_result_reader_poll(
                                 &db_event_id, &db_person, &db_conf_pct,
                                 db_asset, sizeof(db_asset));

         if (!db_inference_valid)
         {
            LOG("[CONTROLLER] waiting_for_result device_id=%d",
                doorbell_device_id);

            pthread_mutex_lock(&g_db_pending_mutex);
            g_db_pending        = 1;
            g_db_pending_device = doorbell_device_id;
            g_db_pending_since  = time(NULL);
            pthread_mutex_unlock(&g_db_pending_mutex);
         }
      }

      /* Inject pending result into bundle if ready and no new press this cycle */
      doorbell_inject_pending(&doorbell_pressed, &doorbell_device_id,
                              &db_inference_valid, &db_event_id,
                              &db_person, &db_conf_pct,
                              db_asset, DOORBELL_ASSET_LEN);

      get_snapshot(&snap);

      age_pir      = snap.age_pir;
      age_lgt      = snap.age_lgt;
      age_lck      = snap.age_lck;
      batt_pir     = snap.batt_pir;
      batt_lck     = snap.batt_lck;
      motor_online = snap.motor_online;
      batt_motor   = snap.batt_motor;

      reed_count = 0;
      for (i = 0; i < MAX_REEDS; i++)
      {
         if (snap.reed_slots[i].active)
         {
            r_state[i] = snap.reed_slots[i].state;
            reed_count = i + 1;
         }
         else
         {
            r_state[i] = UART_STATE_UNKNOWN;
         }
         r_batt[i] = snap.reed_slots[i].batt;
         r_age[i]  = snap.reed_slots[i].age;
      }

      pir_count = 0;
      for (i = 0; i < MAX_PIRS; i++)
      {
         p_count[i]  = snap.pir_slots[i].count;
         p_batt[i]   = snap.pir_slots[i].batt;
         p_age[i]    = snap.pir_slots[i].age;
         p_active[i] = snap.pir_slots[i].active;
         p_occ[i]    = snap.pir_slots[i].occupied;
         if (snap.pir_slots[i].active) { pir_count = snap.pir_count; }
      }

      temp_count = 0;
      for (i = 0; i < MAX_TEMPS; i++)
      {
         t_decidegc[i] = snap.temp_slots[i].temp_decidegc;
         t_batt[i]     = snap.temp_slots[i].batt;
         t_age[i]      = snap.temp_slots[i].age;
         t_active[i]   = snap.temp_slots[i].active;
         if (snap.temp_slots[i].active) { temp_count = snap.temp_count; }
      }

      LOG("[PUSH] ages pir=%u lgt=%u lck=%u | batt pir=%d%% lck=%d%% mtr=%d%% "
          "| reeds=%d pirs=%d temps=%d motor=%d",
          age_pir, age_lgt, age_lck, batt_pir, batt_lck, batt_motor,
          reed_count, pir_count, temp_count, motor_online);

      build_and_push(temp, motion, lgt, lck,
                     age_pir, age_lgt, age_lck,
                     batt_pir, batt_lck, batt_motor,
                     reed_count, motor_online,
                     r_state, r_batt, r_age,
                     pir_count, p_count, p_batt, p_age, p_active, p_occ,
                     temp_count, t_decidegc, t_batt, t_age, t_active,
                     doorbell_pressed, doorbell_device_id,
                     db_inference_valid, db_event_id, db_person, db_conf_pct,
                     db_asset);
   }

   LOG("[PUSH] Push thread exiting");
   return NULL;
}

/******************************************************************************
 * \brief Sync TCP lock state into the UART lock state machine.
 *
 * \param state - New lock state value to set.
 *
 * \return void
 *
 * \details Guards g_lock_state with g_lock_mutex. state_registry owns all
 *          other sensor state — g_lock_state is separate because it drives
 *          the lock transition logic before the resolved state is written
 *          back to the registry via uart_stage_lock().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_sync_lock_state(int state)
{
   pthread_mutex_lock(&g_lock_mutex);
   g_lock_state = (LOCK_STATE_E)state;
   pthread_mutex_unlock(&g_lock_mutex);
}

/******************************************************************************
 * \brief Synchronous outbound push to STM32 from TCP ingress path.
 *
 * \param p_snapshot  - Frozen read model from get_snapshot(). Not modified.
 * \param p_raw_frame - Raw wire frame from sensor pipe. Used for
 *                      doorbell_slots[] (DB<n> per-cam liveness lines).
 *
 * \return void
 *
 * \details Called from sensor_frame_dispatch() on every TCP ingress frame
 *          after update_snapshot() and get_snapshot() complete. Builds the
 *          full STM32 protocol payload from the frozen snapshot and sends
 *          it via uart_push_msg(). Doorbell state is read from shm_data
 *          under shm_mutex and cleared one-shot.
 *
 *          doorbell_pending_check() polls for a late-arriving result;
 *          doorbell_inject_pending() injects it into the bundle if ready,
 *          so the five-field DOORBELL frame always goes out inside the
 *          normal complete payload — never as a standalone transmission.
 *
 *          uart_push_msg() is the only send primitive — uart_write_string
 *          and uart_printf do not exist in this codebase.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_update_frame(const struct LatestData *p_snapshot,
                       const struct SensorData *p_raw_frame)
{
   char    msg[UART_MSG_BUF_SIZE] = {0};
   int     pos                    = 0;
   int     i                      = 0;
   int     reed_count             = 0;
   int      doorbell_pressed       = 0;
   int      doorbell_device_id     = 0;
   uint64_t db_event_id            = 0;
   int      db_inference_valid     = 0;
   uint8_t  db_person              = 0;
   uint8_t  db_conf_pct            = 0;
   char     db_asset[DOORBELL_ASSET_LEN];

   if (!p_snapshot->valid) { return; }
   if (0 > g_uart_fd)      { return; }

   db_asset[0] = '\0';

   /* Compute reed_count from snapshot */
   for (i = 0; i < MAX_REEDS; i++)
   {
      if (p_snapshot->reed_slots[i].active) { reed_count = i + 1; }
   }

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "STATE:%d,%d,%d,%d,%d,%d,%d,%d\n",
                   (int)p_snapshot->avg_temp,
                   p_snapshot->motion_count,
                   p_snapshot->light_state,
                   p_snapshot->lock_state,
                   p_snapshot->age_pir,
                   p_snapshot->age_lgt,
                   p_snapshot->age_lck,
                   reed_count);

   if (p_snapshot->batt_pir >= 0)
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "PIR:%d,%d\n",
                      p_snapshot->motion_count, p_snapshot->batt_pir);
   }
   else
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "PIR:%d\n", p_snapshot->motion_count);
   }

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "PIR_COUNT:%d\n", p_snapshot->pir_count);

   for (i = 0; i < MAX_PIRS; i++)
   {
      if (!p_snapshot->pir_slots[i].active) { continue; }
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "PIR%d:%u,%d,%d\n",
                      i + 1,
                      (unsigned)p_snapshot->pir_slots[i].count,
                      p_snapshot->pir_slots[i].batt,
                      p_snapshot->pir_slots[i].age);
   }

   for (i = 0; i < MAX_PIRS; i++)
   {
      if (!p_snapshot->pir_slots[i].active) { continue; }
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "OCC%d:%d\n", i + 1,
                      p_snapshot->pir_slots[i].occupied);
   }

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "REED_COUNT:%d\n", reed_count);

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (!p_snapshot->reed_slots[i].active) { continue; }
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "DR%d:%d,%d,%d\n",
                      i + 1,
                      p_snapshot->reed_slots[i].state,
                      p_snapshot->reed_slots[i].batt,
                      p_snapshot->reed_slots[i].age);
   }

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "LGT:%d\n", p_snapshot->light_state);

   if (p_snapshot->batt_lck >= 0)
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "LCK:%d,%d\n",
                      p_snapshot->lock_state, p_snapshot->batt_lck);
   }
   else
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "LCK:%d\n", p_snapshot->lock_state);
   }

   if (p_snapshot->batt_motor > 0)
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "MTR:%d,%d\n",
                      p_snapshot->motor_online, p_snapshot->batt_motor);
   }
   else
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "MTR:%d\n", p_snapshot->motor_online);
   }

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "TEMP_COUNT:%d\n", p_snapshot->temp_count);

   for (i = 0; i < MAX_TEMPS; i++)
   {
      if (!p_snapshot->temp_slots[i].active) { continue; }
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "TEMP%d:%d,%d,%d\n",
                      i + 1,
                      p_snapshot->temp_slots[i].temp_decidegc,
                      p_snapshot->temp_slots[i].batt,
                      p_snapshot->temp_slots[i].age);
   }

   /* Doorbell per-cam liveness — mirrors TEMP<n> pattern */
   if (NULL != p_raw_frame)
   {
      for (i = 0; i < MAX_DOORBELL_CAMS; i++)
      {
         pos += snprintf(msg + pos, sizeof(msg) - pos,
                         "DB%d:%d,%d\n",
                         i,
                         p_raw_frame->doorbell_slots[i].age_s,
                         p_raw_frame->doorbell_slots[i].online);
      }
   }

   /* Check for late-arriving result from a previous press cycle */
   doorbell_pending_check();

   /* Read doorbell press event from shm one-shot — clear after reading.
    * shm_mutex released before polling inference result below.          */
   pthread_mutex_lock(&shm_data->shm_mutex);
   doorbell_pressed   = shm_data->doorbell_pressed;
   doorbell_device_id = shm_data->doorbell_device_id;
   if (doorbell_pressed) { shm_data->doorbell_pressed = 0; }
   pthread_mutex_unlock(&shm_data->shm_mutex);

   /* Poll inference result from doorbell_daemon's /doorbell_result shm.
    * Called outside shm_mutex — no lock held. Only meaningful on a real
    * press; skip entirely otherwise to avoid consuming an event_id that
    * belongs to the next press.                                          */
   if (0 != doorbell_pressed)
   {
      db_inference_valid = doorbell_result_reader_poll(
                              &db_event_id, &db_person, &db_conf_pct,
                              db_asset, sizeof(db_asset));

      if (!db_inference_valid)
      {
         LOG("[CONTROLLER] waiting_for_result device_id=%d", doorbell_device_id);
         pthread_mutex_lock(&g_db_pending_mutex);
         g_db_pending        = 1;
         g_db_pending_device = doorbell_device_id;
         g_db_pending_since  = time(NULL);
         pthread_mutex_unlock(&g_db_pending_mutex);
      }
   }

   /* Inject pending result into bundle if ready and no new press this cycle */
   doorbell_inject_pending(&doorbell_pressed, &doorbell_device_id,
                           &db_inference_valid, &db_event_id,
                           &db_person, &db_conf_pct,
                           db_asset, DOORBELL_ASSET_LEN);

   /* Emit extended five-field frame when inference result is available,
    * two-field fallback otherwise — STM32 parser handles both forms.   */
   if (doorbell_pressed && db_inference_valid)
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "DOORBELL:%d,%d,%d,%d,%s\n",
                      doorbell_pressed, doorbell_device_id,
                      db_person, db_conf_pct, db_asset);
      LOG("[CONTROLLER] -> [UART] send event_id=" EVENT_ID_FMT
          " device_id=%d person=%d conf=%d",
          EVENT_ID_ARG(db_event_id), doorbell_device_id,
          db_person, db_conf_pct);
   }
   else
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "DOORBELL:%d,%d\n",
                      doorbell_pressed, doorbell_device_id);
   }

   /* Inference cam liveness — derived from shm at push time */
   pthread_mutex_lock(&shm_data->shm_mutex);
   for (i = 0; i < CAM_COUNT; i++)
   {
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "CAM%d:%d\n", i + 1,
                      shm_data->cam_online[i]);
   }
   pthread_mutex_unlock(&shm_data->shm_mutex);

   uart_push_msg(msg, pos);
}
