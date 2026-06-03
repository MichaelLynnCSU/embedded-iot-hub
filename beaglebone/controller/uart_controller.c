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
 *          latest_data.pir_slots[i].occupied. build_and_push() emits
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
 ******************************************************************************/

#include <termios.h>
#include "controller_internal.h"
#include "controller_logic.h"

#define UART_PUSH_INTERVAL_SEC  5       /**< max wait in sem_timedwait — watchdog ceiling */
#define UART_RETRY_DELAY_SEC    5       /**< delay before retrying UART open */
#define UART_PUSH_DELAY_US      100000  /**< delay after write in microseconds */
#define UART_MSG_BUF_SIZE       1024     /**< push message buffer size bytes */
#define UART_STATE_UNKNOWN      0xFF    /**< sentinel for unknown reed state */
#define UART_BAUD               B115200 /**< UART baud rate */

static int             g_uart_fd          = -1;  /**< UART file descriptor */
static pthread_mutex_t g_uart_write_mutex =
   PTHREAD_MUTEX_INITIALIZER;                     /**< UART write mutex */

/** \brief Lock state machine — owned here, protected by data_mutex */
static LOCK_STATE_E g_lock_state = LOCK_STATE_LOCKED;

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
   unsigned next_head = 0; /**< candidate new head index */

   pthread_mutex_lock(&g_uart_ring.mutex);

   next_head = (g_uart_ring.head + 1) & (UART_RING_SIZE - 1);

   if (next_head == g_uart_ring.tail)
   {
      /* Ring full — advance tail to drop the oldest frame */
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
      /* Should never happen — sem count and ring depth are always in sync */
      LOG_WRN("[RING] Pop called on empty ring — semaphore mismatch");
      return 0;
   }

   *p_frame         = g_uart_ring.frames[g_uart_ring.tail];
   g_uart_ring.tail = (g_uart_ring.tail + 1) & (UART_RING_SIZE - 1);

   return 1;
}

/******************************************************************************
 * \brief Open UART device at 115200 8N1.
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
 ******************************************************************************/
static void uart_process_lock(int val, int batt)
{
   LOCK_STATE_E    old_state = LOCK_STATE_LOCKED;
   LOCK_STATE_E    new_state = LOCK_STATE_LOCKED;
   const char     *p_ev      = NULL;

   pthread_mutex_lock(&data_mutex);

   old_state = g_lock_state;
   new_state = logic_lock_transition(old_state, val);

   if (new_state == old_state)
   {
      if (logic_lock_is_busy(old_state))
      {
         LOG("[LCK] Command rejected — motor moving (%s)",
             logic_lock_state_label(old_state));
      }
      pthread_mutex_unlock(&data_mutex);
      return;
   }

   g_lock_state            = new_state;
   latest_data.lock_state  = (int)new_state;
   latest_data.batt_lck    = (int8_t)batt;

   pthread_mutex_unlock(&data_mutex);

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
 * \details After updating latest_data, pushes a UartFrame onto g_uart_ring
 *          and calls sem_post(&g_uart_frame_sem). The semaphore count
 *          therefore equals the number of frames in the ring buffer at all
 *          times, which is the invariant uart_push_thread relies on.
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
   UartFrame   frame    = {0};              /**< frame pushed to ring buffer */
   struct CommandMsg auto_cmd = {.cmd = CMD_GET_LATEST};

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
   else
   {
      pthread_mutex_lock(&data_mutex);

      if (DEV_PIR == idx)
      {
         latest_data.motion_count = val;
         latest_data.valid        = 1;
      }
      else if (DEV_LIGHT == idx)
      {
         latest_data.light_state = val;
      }

      pthread_mutex_unlock(&data_mutex);
   }

   /* Push frame onto ring buffer then signal the consumer.
    * Order is mandatory: push before post so the frame is visible
    * to uart_push_thread the moment it wakes from sem_timedwait.
    * Semaphore value after post == number of frames in the ring. */
   frame.idx  = idx;
   frame.val  = val;
   frame.batt = batt;
   uart_ring_push(&frame);
   sem_post(&g_uart_frame_sem);

   handle_get_latest(&auto_cmd);
}

/******************************************************************************
 * \brief Thread — reads line-framed UART data from STM32.
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
 * \brief Snapshot reed slot state from latest_data under data_mutex.
 ******************************************************************************/
static void snapshot_reed_slots(uint8_t  *p_r_state,
                                 int8_t   *p_r_batt,
                                 uint16_t *p_r_age,
                                 int      *p_reed_count)
{
   int i = 0;

   pthread_mutex_lock(&data_mutex);

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (latest_data.reed_slots[i].active)
      {
         p_r_state[i]  = latest_data.reed_slots[i].state;
         *p_reed_count = i + 1;
      }
      else
      {
         p_r_state[i] = UART_STATE_UNKNOWN;
      }

      p_r_batt[i] = latest_data.reed_slots[i].batt;
      p_r_age[i]  = latest_data.reed_slots[i].age;
   }

   pthread_mutex_unlock(&data_mutex);
}

/******************************************************************************
 * \brief Snapshot PIR slot state from latest_data under data_mutex.
 ******************************************************************************/
static void snapshot_pir_slots(uint32_t *p_count,
                                int8_t   *p_batt,
                                uint16_t *p_age,
                                uint8_t  *p_active,
                                int      *p_occ,
                                int      *p_pir_count)
{
   int i = 0;

   pthread_mutex_lock(&data_mutex);

   *p_pir_count = 0;

   for (i = 0; i < MAX_PIRS; i++)
   {
      p_count[i]  = latest_data.pir_slots[i].count;
      p_batt[i]   = latest_data.pir_slots[i].batt;
      p_age[i]    = latest_data.pir_slots[i].age;
      p_active[i] = latest_data.pir_slots[i].active;
      p_occ[i]    = latest_data.pir_slots[i].occupied;

      if (latest_data.pir_slots[i].active)
      {
         *p_pir_count = latest_data.pir_count;
      }
   }

   pthread_mutex_unlock(&data_mutex);
}

static void snapshot_temp_slots(int16_t  *p_count,
                                 int8_t   *p_batt,
                                 uint16_t *p_age,
                                 uint8_t  *p_active,
                                 int      *p_temp_count)
{
   int i = 0;

   pthread_mutex_lock(&data_mutex);

   *p_temp_count = 0;

   for (i = 0; i < MAX_TEMPS; i++)
   {
      p_count[i]  = latest_data.temp_slots[i].temp_decidegc;
      p_batt[i]   = latest_data.temp_slots[i].batt;
      p_age[i]    = latest_data.temp_slots[i].age;
      p_active[i] = latest_data.temp_slots[i].active;

      if (latest_data.temp_slots[i].active)
      {
         *p_temp_count = latest_data.temp_count;
      }
   }

   pthread_mutex_unlock(&data_mutex);
}

/******************************************************************************
 * \brief Build UART push payload and send to STM32.
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
                            const uint8_t  *p_tactive)

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

   uart_push_msg(msg, pos);
}

/******************************************************************************
 * \brief Thread — sends sensor state to STM32, driven by g_uart_frame_sem.
 *
 * \details Replaces the previous sleep(UART_PUSH_INTERVAL_SEC) polling loop.
 *          sem_timedwait blocks with zero CPU until uart_parse_line() posts
 *          a new frame onto g_uart_ring, or until UART_PUSH_INTERVAL_SEC
 *          elapses as a watchdog ceiling (preserves existing heartbeat
 *          push behavior when UART is quiet).
 *
 *          After waking, pops all pending frames from g_uart_ring to drain
 *          any burst that arrived while the previous push was executing.
 *          The pop count is logged for burst visibility. latest_data is
 *          then snapshotted for the consolidated push to STM32.
 *
 *          shm_data is read under shm_data->shm_mutex (process-shared),
 *          replacing the previous sem_timedwait(shm_sem) pattern.
 ******************************************************************************/
void *uart_push_thread(void *p_arg)
{
   int      valid        = 0;
   double   temp         = 0.0;
   int      motion       = 0;
   int      lgt          = 0;
   int      lck          = 0;
   uint16_t age_pir      = 0;
   uint16_t age_lgt      = 0;
   uint16_t age_lck      = 0;
   int8_t   batt_pir     = -1;
   int8_t   batt_lck     = -1;
   int      batt_motor   = -1;
   int      motor_online = 0;
   int      reed_count   = 0;
   int      pir_count    = 0;
   int      frames_ready = 0; /**< frames drained from ring this wake */
   UartFrame frame       = {0};

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
   int      temp_count = 0;

   (void)p_arg;

   LOG("[PUSH] Push thread started (sem-driven, ceiling=%ds)",
       UART_PUSH_INTERVAL_SEC);

   while (running)
   {
      /* Block until uart_parse_line() signals a new frame, or the
       * watchdog ceiling expires. UART_PUSH_INTERVAL_SEC preserves
       * the original worst-case push cadence for quiet periods.      */
      {
         struct timespec deadline;
         clock_gettime(CLOCK_REALTIME, &deadline);
         deadline.tv_sec += UART_PUSH_INTERVAL_SEC;
         sem_timedwait(&g_uart_frame_sem, &deadline);
      }

      if (!running) { break; }

      if (0 > g_uart_fd) { continue; }

      /* Drain all frames that arrived since the last push.
       * Each successful pop decrements the semaphore count by one
       * (already decremented by sem_timedwait for the first frame).
       * frames_ready tells us the burst depth — logged for visibility. */
      frames_ready = 0;
      if (uart_ring_pop(&frame)) { frames_ready = 1; }
      while (sem_trywait(&g_uart_frame_sem) == 0)
      {
         uart_ring_pop(&frame); /* keep frame = newest in burst */
         frames_ready++;
      }

      LOG("[PUSH] Woke: %d UART frame(s) in burst", frames_ready);

      /* Read shm_data under the process-shared mutex.
       * Replaces the previous sem_timedwait(shm_sem) pattern. */
      pthread_mutex_lock(&shm_data->shm_mutex);
      valid  = shm_data->data_valid;
      temp   = shm_data->current_temp;
      motion = shm_data->current_motion;
      lgt    = shm_data->current_light;
      lck    = shm_data->current_lock;
      pthread_mutex_unlock(&shm_data->shm_mutex);

      if (!valid)
      {
         LOG("[PUSH] No valid data yet, skipping");
         continue;
      }

      pthread_mutex_lock(&data_mutex);
      age_pir      = latest_data.age_pir;
      age_lgt      = latest_data.age_lgt;
      age_lck      = latest_data.age_lck;
      batt_pir     = latest_data.batt_pir;
      batt_lck     = latest_data.batt_lck;
      motor_online = latest_data.motor_online;
      batt_motor   = latest_data.batt_motor;
      pthread_mutex_unlock(&data_mutex);

      reed_count = 0;
      snapshot_reed_slots(r_state, r_batt, r_age, &reed_count);

      pir_count = 0;
      snapshot_pir_slots(p_count, p_batt, p_age, p_active, p_occ, &pir_count);

      temp_count = 0;
      snapshot_temp_slots(t_decidegc, t_batt, t_age, t_active, &temp_count);

      LOG("[PUSH] ages pir=%u lgt=%u lck=%u | batt pir=%d%% lck=%d%% mtr=%d%% "
          "| reeds=%d pirs=%d motor=%d",
          age_pir, age_lgt, age_lck, batt_pir, batt_lck, batt_motor,
          reed_count, pir_count, motor_online);

      build_and_push(temp, motion, lgt, lck,
                     age_pir, age_lgt, age_lck,
                     batt_pir, batt_lck, batt_motor,
                     reed_count, motor_online,
                     r_state, r_batt, r_age,
                     pir_count, p_count, p_batt, p_age, p_active, p_occ,
                     temp_count, t_decidegc, t_batt, t_age, t_active);
   }

   LOG("[PUSH] Push thread exiting");
   return NULL;
}

/******************************************************************************
 * \brief Sync TCP lock state into the UART lock state machine.
 ******************************************************************************/
void uart_sync_lock_state(int state)
{
   pthread_mutex_lock(&data_mutex);
   g_lock_state = (LOCK_STATE_E)state;
   pthread_mutex_unlock(&data_mutex);
}
