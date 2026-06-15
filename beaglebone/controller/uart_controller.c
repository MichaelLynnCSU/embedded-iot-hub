/******************************************************************************
 * \file uart_controller.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-15
 *
 * \brief UART controller — thin coordinator after 2026-06-15 module split.
 *
 * \details After the split, this file owns only:
 *            - g_uart_frame_sem and g_uart_ring definitions (extern in
 *              uart_controller.h, used by both uart_transport.c and here)
 *            - build_and_push()  — payload formatter, calls uart_push_msg()
 *            - uart_push_thread() — UART-ingress push path (sem-driven)
 *            - uart_update_frame() — TCP-ingress push path (synchronous)
 *
 *          Implementations moved to dedicated modules (no behavior change):
 *            uart_transport.c  — uart_reader_thread, uart_ring_push/pop,
 *                                uart_push_msg, uart_transport_is_open
 *            uart_lock.c       — uart_process_lock, uart_sync_lock_state,
 *                                g_lock_state / g_lock_mutex
 *            doorbell_pending.c — doorbell_pending_mark,
 *                                 doorbell_pending_check,
 *                                 doorbell_inject_pending,
 *                                 g_db_pending* state
 *
 *          The "doorbell pending" calls in both push paths previously wrote
 *          g_db_pending* directly — now they call doorbell_pending_mark()
 *          from doorbell_pending.h. The fd guard previously checked
 *          g_uart_fd directly — now calls uart_transport_is_open().
 *
 * \note    See the original uart_controller.c header (pre-split) for the
 *          full design rationale, protocol format, and change history.
 ******************************************************************************/

#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include "config.h"
#include "sensor_types.h"
#include "shared_data.h"
#include "log.h"
#include "globals.h"
#include "uart_controller.h"
#include "uart_transport.h"
#include "uart_lock.h"
#include "doorbell_pending.h"
#include "cmd/shm_updater.h"
#include "cmd/state_registry.h"
#include "doorbell_result_reader.h"
#include "doorbell_result_shm.h"

#define DOORBELL_ASSET_LEN      20      /**< matches DoorbellResult.asset[20] */
#define UART_PUSH_INTERVAL_SEC  5       /**< sem_timedwait watchdog ceiling   */
#define UART_MSG_BUF_SIZE       1024    /**< push message buffer bytes        */
#define UART_STATE_UNKNOWN      0xFF    /**< sentinel for unknown reed state  */

/* g_uart_frame_sem and g_uart_ring are defined in data_controller.c
 * and declared extern in uart_controller.h — no definition here.     */

/******************************************************************************
 * \brief Build UART push payload from explicit slot arrays and send to STM32.
 *
 * \details Called by uart_push_thread() only. Takes pre-extracted slot
 *          arrays rather than a snapshot struct so the thread can apply
 *          its own doorbell read/clear logic before calling here.
 *
 *          When db_inference_valid is non-zero the extended five-field
 *          DOORBELL frame is emitted; otherwise the two-field fallback is
 *          used. The STM32 parser handles both forms.
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
 *          frames are handled synchronously by uart_update_frame().
 *
 *          Doorbell state is read from shm_data under shm_mutex and cleared
 *          one-shot. doorbell_pending_check() polls for a late-arriving
 *          result; doorbell_inject_pending() injects it into the bundle if
 *          ready, so the five-field DOORBELL frame always goes out inside
 *          the normal complete payload.
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

      /* uart_transport_is_open() replaces the old direct g_uart_fd check */
      if (!uart_transport_is_open()) { continue; }

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
            /* doorbell_pending_mark() replaces direct g_db_pending* writes */
            doorbell_pending_mark(doorbell_device_id);
         }
      }

      /* Inject pending result into bundle if ready and no new press */
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
 * \brief Synchronous outbound push to STM32 from TCP ingress path.
 *
 * \param p_snapshot  - Frozen read model from get_snapshot(). Not modified.
 * \param p_raw_frame - Raw wire frame from sensor pipe. Used for
 *                      doorbell_slots[] (DB<n> per-cam liveness lines).
 *
 * \return void
 *
 * \details Called from sensor_frame_dispatch() on every TCP ingress frame.
 *          Builds the full STM32 protocol payload from the frozen snapshot
 *          and sends it via uart_push_msg(). Doorbell state is read from
 *          shm_data under shm_mutex and cleared one-shot.
 *
 *          doorbell_pending_check() polls for a late-arriving result;
 *          doorbell_inject_pending() injects it into the bundle if ready.
 *          doorbell_pending_mark() records a press when no result is
 *          available yet — replaces direct g_db_pending* writes.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_update_frame(const struct LatestData *p_snapshot,
                       const struct SensorData *p_raw_frame)
{
   char     msg[UART_MSG_BUF_SIZE] = {0};
   int      pos                    = 0;
   int      i                      = 0;
   int      reed_count             = 0;
   int      doorbell_pressed       = 0;
   int      doorbell_device_id     = 0;
   uint64_t db_event_id            = 0;
   int      db_inference_valid     = 0;
   uint8_t  db_person              = 0;
   uint8_t  db_conf_pct            = 0;
   char     db_asset[DOORBELL_ASSET_LEN];

   if (!p_snapshot->valid)        { return; }
   if (!uart_transport_is_open()) { return; }  /* replaces: 0 > g_uart_fd */

   db_asset[0] = '\0';

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

   pthread_mutex_lock(&shm_data->shm_mutex);
   doorbell_pressed   = shm_data->doorbell_pressed;
   doorbell_device_id = shm_data->doorbell_device_id;
   if (doorbell_pressed) { shm_data->doorbell_pressed = 0; }
   pthread_mutex_unlock(&shm_data->shm_mutex);

   if (0 != doorbell_pressed)
   {
      db_inference_valid = doorbell_result_reader_poll(
                              &db_event_id, &db_person, &db_conf_pct,
                              db_asset, sizeof(db_asset));

      if (!db_inference_valid)
      {
         LOG("[CONTROLLER] waiting_for_result device_id=%d", doorbell_device_id);
         /* doorbell_pending_mark() replaces direct g_db_pending* writes */
         doorbell_pending_mark(doorbell_device_id);
      }
   }

   /* Inject pending result into bundle if ready and no new press */
   doorbell_inject_pending(&doorbell_pressed, &doorbell_device_id,
                           &db_inference_valid, &db_event_id,
                           &db_person, &db_conf_pct,
                           db_asset, DOORBELL_ASSET_LEN);

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
