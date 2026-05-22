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
 *          parses device frames, stamps heartbeats, saves to DB.
 *
 *          uart_push_thread — sends consolidated sensor state to STM32
 *          every UART_PUSH_INTERVAL_SEC seconds.
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
 ******************************************************************************/

#include <termios.h>
#include "controller_internal.h"
#include "controller_logic.h"

#define UART_PUSH_INTERVAL_SEC  5       /**< push thread send interval seconds */
#define UART_RETRY_DELAY_SEC    5       /**< delay before retrying UART open */
#define UART_PUSH_DELAY_US      100000  /**< delay after write in microseconds */
#define UART_MSG_BUF_SIZE       512     /**< push message buffer size bytes */
#define UART_STATE_UNKNOWN      0xFF    /**< sentinel for unknown reed state */
#define UART_BAUD               B115200 /**< UART baud rate */

static int             g_uart_fd          = -1;  /**< UART file descriptor */
static pthread_mutex_t g_uart_write_mutex =
   PTHREAD_MUTEX_INITIALIZER;                     /**< UART write mutex */

/** \brief Lock state machine — owned here, protected by data_mutex */
static LOCK_STATE_E g_lock_state = LOCK_STATE_LOCKED;

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
 *
 * \param p_count     - Output array of motion counts, size MAX_PIRS.
 * \param p_batt      - Output array of battery SOC, size MAX_PIRS.
 * \param p_age       - Output array of ages in seconds, size MAX_PIRS.
 * \param p_active    - Output array of active flags, size MAX_PIRS.
 * \param p_occ       - Output array of per-slot occupied flags, size MAX_PIRS.
 * \param p_pir_count - Output for number of active PIR slots.
 *
 * \note  Per-slot OCC (2026-05-20): p_occ[] added, populated from
 *        latest_data.pir_slots[i].occupied set by data_controller.c
 *        when it processes the ESP32 JSON pirs[].occupied field.
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
         *p_pir_count = i + 1;
      }
   }

   pthread_mutex_unlock(&data_mutex);
}

/******************************************************************************
 * \brief Build UART push payload and send to STM32.
 *
 * \note  Per-slot OCC (2026-05-20): p_pocc[] added. Single OCC:n global
 *        frame replaced by OCC<n>:n per-slot frames emitted after PIR<n>
 *        lines. STM32 parser handles OCC<1-4> identically to PIR<1-4>.
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
                            const int      *p_pocc)
{
   char msg[UART_MSG_BUF_SIZE] = {0};
   int  pos                    = 0;
   int  i                      = 0;

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "STATE:%d,%d,%d,%d,%d,%d,%d,%d\n",
                   (int)temp, motion, lgt, lck,
                   age_pir, age_lgt, age_lck, reed_count);

   /* Legacy flat PIR line — preserved for STM32 backward compatibility */
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

   /* Per-slot PIR lines — mirrors reed DR<n> pattern */
   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "PIR_COUNT:%d\n", pir_count);

   for (i = 0; i < pir_count; i++)
   {
      if (!p_pactive[i]) { continue; }

      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "PIR%d:%u,%d,%d\n",
                      i + 1, (unsigned)p_pc[i], p_pb[i], p_pa[i]);
   }

   /* Per-slot OCC frames (2026-05-20) — replaces single OCC:n */
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

   uart_push_msg(msg, pos);
}

/******************************************************************************
 * \brief Thread — sends sensor state to STM32 every UART_PUSH_INTERVAL_SEC.
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

   uint8_t  r_state[MAX_REEDS];
   int8_t   r_batt[MAX_REEDS];
   uint16_t r_age[MAX_REEDS];

   uint32_t p_count[MAX_PIRS];  /**< PIR motion count snapshot  */
   int8_t   p_batt[MAX_PIRS];   /**< PIR battery snapshot       */
   uint16_t p_age[MAX_PIRS];    /**< PIR age snapshot           */
   uint8_t  p_active[MAX_PIRS]; /**< PIR active flags snapshot  */
   int      p_occ[MAX_PIRS];    /**< PIR per-slot occupied flags — 2026-05-20 */

   (void)p_arg;

   LOG("[PUSH] Push thread started (interval=%ds)", UART_PUSH_INTERVAL_SEC);

   while (running)
   {
      sleep(UART_PUSH_INTERVAL_SEC);

      if (0 > g_uart_fd) { continue; }

      {
         struct timespec ts;
         clock_gettime(CLOCK_REALTIME, &ts);
         ts.tv_sec += 2;

         if (0 != sem_timedwait(shm_sem, &ts))
         {
            LOG("[PUSH] shm_sem timeout -- web process may be dead, skipping");
            continue;
         }
      }

      valid  = shm_data->data_valid;
      temp   = shm_data->current_temp;
      motion = shm_data->current_motion;
      lgt    = shm_data->current_light;
      lck    = shm_data->current_lock;
      sem_post(shm_sem);

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

      LOG("[PUSH] ages pir=%u lgt=%u lck=%u | batt pir=%d%% lck=%d%% mtr=%d%% "
          "| reeds=%d pirs=%d motor=%d",
          age_pir, age_lgt, age_lck, batt_pir, batt_lck, batt_motor,
          reed_count, pir_count, motor_online);

      build_and_push(temp, motion, lgt, lck,
                     age_pir, age_lgt, age_lck,
                     batt_pir, batt_lck, batt_motor,
                     reed_count, motor_online,
                     r_state, r_batt, r_age,
                     pir_count, p_count, p_batt, p_age, p_active, p_occ);
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
