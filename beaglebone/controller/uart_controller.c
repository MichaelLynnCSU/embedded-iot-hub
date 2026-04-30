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
 *          REED_COUNT:n\n
 *          DR<n>:state,batt,age\n  (one per active reed slot)
 *          LGT:state\n
 *          LCK:state[,batt]\n
 *          MTR:online\n
 *
 *          Lock state machine (LOCK_STATE_E from controller_logic.h):
 *          Inbound LCK frames drive logic_lock_transition(). Commands
 *          are rejected while motor is moving (UNLOCKING/LOCKING).
 *          All transitions are logged to device_events via db_save_event().
 *
 *          Adding ReedSensor3..N requires zero code changes here —
 *          ESP32 auto-discovers, reed_count increments in JSON,
 *          STM32 receives REED_COUNT:n and calls UI_Reflow(n).
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
 *
 * \param p_dev - Null-terminated device path string.
 *
 * \return int - File descriptor on success, -1 on failure.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static int uart_open(const char *p_dev)
{
   int            fd  = -1;  /**< file descriptor */
   struct termios tty;       /**< terminal settings */

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
 * \param val  - Lock value from STM32 (0=locked, 1=unlocked).
 * \param batt - Battery SOC percent, -1 if absent.
 *
 * \return void
 *
 * \details Runs logic_lock_transition() to validate the command.
 *          If state changes, logs transition to device_events and
 *          updates latest_data.lock_state. Rejects commands while
 *          motor is moving (UNLOCKING or LOCKING states).
 *          Must NOT be called with data_mutex held.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void uart_process_lock(int val, int batt)
{
   LOCK_STATE_E    old_state = LOCK_STATE_LOCKED; /**< previous state */
   LOCK_STATE_E    new_state = LOCK_STATE_LOCKED; /**< new state */
   const char     *p_ev      = NULL;              /**< event string */

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
 * \param p_line - Null-terminated frame string without newline.
 *
 * \return void
 *
 * \details Frame format: <ID>:<value>[,<batt>]
 *          Valid IDs: PIR, LGT, LCK
 *          Stamps heartbeat, saves to DB, updates latest_data.
 *          LCK frames are routed through uart_process_lock() for
 *          state machine validation before updating latest_data.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void uart_parse_line(const char *p_line)
{
   char        buf[UART_LINE_LEN] = {0}; /**< local copy of line */
   char       *p_colon  = NULL;          /**< pointer to colon separator */
   char       *p_rest   = NULL;          /**< pointer to value portion */
   char       *p_comma  = NULL;          /**< pointer to comma separator */
   const char *p_id     = NULL;          /**< device ID string */
   int         val      = 0;             /**< parsed sensor value */
   int         batt     = -1;            /**< parsed battery SOC, -1=absent */
   DEV_ID_E    idx      = (DEV_ID_E)-1; /**< device index */
   struct CommandMsg auto_cmd = {.cmd = CMD_GET_LATEST}; /**< auto command */

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
      /* LCK — route through state machine, updates latest_data internally */
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
 *
 * \param p_arg - Unused thread argument.
 *
 * \return void* - Always returns NULL.
 *
 * \details Opens UART device, reads bytes into a line buffer, and calls
 *          uart_parse_line() on each complete line. Reopens UART on
 *          read error. Runs until running flag is cleared.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void *uart_reader_thread(void *p_arg)
{
   char    line[UART_LINE_LEN] = {0}; /**< line accumulation buffer */
   int     pos                 = 0;   /**< current line buffer position */
   char    c                   = 0;   /**< received byte */
   ssize_t n                   = 0;   /**< bytes read */

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

         if (0 == n)
         {
            continue;
         }

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
 * \param len   - Number of bytes to write.
 *
 * \return void
 *
 * \details Acquires g_uart_write_mutex, writes, releases mutex, then
 *          sleeps UART_PUSH_DELAY_US to allow STM32 to process.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void uart_push_msg(const char *p_msg, int len)
{
   ssize_t w = 0; /**< bytes written */

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
 *
 * \param p_r_state    - Output array of reed states, size MAX_REEDS.
 * \param p_r_batt     - Output array of reed batteries, size MAX_REEDS.
 * \param p_r_age      - Output array of reed ages, size MAX_REEDS.
 * \param p_reed_count - Output for highest active reed slot index + 1.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void snapshot_reed_slots(uint8_t  *p_r_state,
                                 int8_t   *p_r_batt,
                                 uint16_t *p_r_age,
                                 int      *p_reed_count)
{
   int i = 0; /**< loop index */

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
 * \brief Build UART push payload and send to STM32.
 *
 * \param temp         - Current average temperature.
 * \param motion       - Current motion count.
 * \param lgt          - Current light state.
 * \param lck          - Current lock state.
 * \param age_pir      - PIR device age in seconds.
 * \param age_lgt      - Light device age in seconds.
 * \param age_lck      - Lock device age in seconds.
 * \param batt_pir     - PIR battery SOC percent.
 * \param batt_lck     - Lock battery SOC percent.
 * \param batt_motor   - Motor battery SOC percent.
 * \param reed_count   - Number of active reed slots.
 * \param motor_online - Motor controller online flag (0=offline, 1=online).
 * \param occupied     - PIR Sliding window
 * \param p_r_state    - Reed state array, size MAX_REEDS.
 * \param p_r_batt     - Reed battery array, size MAX_REEDS.
 * \param p_r_age      - Reed age array, size MAX_REEDS.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void build_and_push(double temp, int motion, int lgt, int lck,
                            uint16_t age_pir, uint16_t age_lgt,
                            uint16_t age_lck, int8_t batt_pir,
                            int8_t batt_lck, int8_t batt_motor,
                            int reed_count, int motor_online,
                            int occupied,
                            const uint8_t  *p_r_state,
                            const int8_t   *p_r_batt,
                            const uint16_t *p_r_age)
{
   char msg[UART_MSG_BUF_SIZE] = {0}; /**< outbound message buffer */
   int  pos                    = 0;   /**< current buffer position */
   int  i                      = 0;   /**< loop index */

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
                   "OCC:%d\n", occupied);

   pos += snprintf(msg + pos, sizeof(msg) - pos,
                   "REED_COUNT:%d\n", reed_count);

   for (i = 0; i < reed_count; i++)
   {
      if (UART_STATE_UNKNOWN == p_r_state[i])
      {
         continue;
      }

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
      pos += snprintf(msg + pos, sizeof(msg) - pos,
                      "LCK:%d\n", lck);
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
 *
 * \param p_arg - Unused thread argument.
 *
 * \return void* - Always returns NULL.
 *
 * \details Snapshots shared memory and latest_data, builds push payload,
 *          and sends via uart_push_msg(). Skips if no valid data or
 *          UART not open. Runs until running flag is cleared.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void *uart_push_thread(void *p_arg)
{
   int      valid        = 0;    /**< shm data valid flag */
   double   temp         = 0.0;  /**< current temperature */
   int      motion       = 0;    /**< current motion count */
   int      lgt          = 0;    /**< current light state */
   int      lck          = 0;    /**< current lock state */
   uint16_t age_pir      = 0;    /**< PIR age seconds */
   uint16_t age_lgt      = 0;    /**< light age seconds */
   uint16_t age_lck      = 0;    /**< lock age seconds */
   int8_t   batt_pir     = -1;   /**< PIR battery SOC */
   int8_t   batt_lck     = -1;   /**< lock battery SOC */
   int      batt_motor   = -1;   /**< motor battery SOC */
   int      motor_online = 0;    /**< motor controller online flag */
   int      reed_count   = 0;    /**< active reed slot count */
   int      occupied     = 0;
   uint8_t  r_state[MAX_REEDS];  /**< reed state snapshot */
   int8_t   r_batt[MAX_REEDS];   /**< reed battery snapshot */
   uint16_t r_age[MAX_REEDS];    /**< reed age snapshot */

   (void)p_arg;

   LOG("[PUSH] Push thread started (interval=%ds)", UART_PUSH_INTERVAL_SEC);

   while (running)
   {
      sleep(UART_PUSH_INTERVAL_SEC);

      if (0 > g_uart_fd)
      {
         continue;
      }

      sem_wait(shm_sem);
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
      occupied     = latest_data.pir_occupied;
      pthread_mutex_unlock(&data_mutex);

      reed_count = 0;
      snapshot_reed_slots(r_state, r_batt, r_age, &reed_count);

      LOG("[PUSH] ages pir=%u lgt=%u lck=%u | batt pir=%d%% lck=%d%% mtr=%d%% | reeds=%d motor=%d occ=%d",
          age_pir, age_lgt, age_lck, batt_pir, batt_lck, batt_motor,
          reed_count, motor_online, occupied);

      build_and_push(temp, motion, lgt, lck,
                     age_pir, age_lgt, age_lck,
                     batt_pir, batt_lck, batt_motor,
                     reed_count, motor_online, occupied,
                     r_state, r_batt, r_age);
   }

   LOG("[PUSH] Push thread exiting");
   return NULL;
}
