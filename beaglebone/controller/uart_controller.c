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
 *            - uart_update_frame() — pipe-ingress push path (synchronous)
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
 * \note    SHM read path (dst=blackpill_lcd):
 *          Both push paths previously read sensor state fields from SHM
 *          (valid, temp, motion, lgt, lck, cam_online). These have been
 *          removed in favour of state_registry.c via get_snapshot() and
 *          p_raw_frame directly (2026-06-19 cleanup):
 *
 *          uart_push_thread() (UART ingress path):
 *            - SHM mutex block now holds only doorbell_pressed consume-
 *              and-clear — one job, minimal hold time.
 *            - valid/temp/motion/lgt/lck come from snap (LatestData)
 *              via get_snapshot(), which is now called before the
 *              doorbell block so snap is populated when needed.
 *            - No semantic change — shm_updater.c projects the same
 *              values from state_registry into both SHM and LatestData.
 *
 *          uart_update_frame() (pipe ingress path):
 *            - cam_online[] loop now reads p_raw_frame->cam_slots[i].online
 *              directly instead of shm_data->cam_online[i].
 *            - Mutex lock/unlock around that loop removed.
 *            - Confirmed equivalent: shm_updater.c line 143 is a direct
 *              assignment shm_data->cam_online[i] = p_data->cam_slots[i].online
 *              with no transformation or liveness logic applied.
 *            - doorbell_pressed consume-and-clear remains under SHM mutex
 *              — load-bearing, cannot be moved to snapshot.
 *
 *          sensor_shm segment itself is unchanged — thermostat_lcd still
 *          reads from it for its own display path.
 *
 * \note    Rate-limit to 2 per second max.
 *          Bundle size grows with doorbell/cam count (~280 bytes at 3DB/4CAM,
 *          ~24ms at 115200 baud). Back-to-back TCP frames from simultaneous
 *          camera triggers can saturate the STM32 ring buffer before
 *          drain_uart_queue() drains it. 500ms gives ~100 drain cycles.
 *
 * \note    Per-device event_id tracing, full pipeline (2026-06-19):
 *          The single rolled-up event_id previously carried in
 *          shm_data->event_id (set by handle_get_latest(), picking
 *          lock_event_id with a light_event_id fallback) has been
 *          removed at the source in shm_updater.c — see that file's
 *          header note "Per-device event_id logging (2026-06-19)" for
 *          why: it silently discarded every PIR/reed/temp event_id to
 *          represent the whole snapshot with one device's ID.
 *
 *          Both push paths in this file previously read
 *          shm_data->event_id back out and stamped it onto the outbound
 *          UartMsg envelope (msg.event_id / msg.has_event_id) and into
 *          a "[SHM] ... event_id=... read dst=blackpill_lcd" log line.
 *          That field no longer exists upstream, so both reads have
 *          been removed along with it.
 *
 *          In its place, both build_and_push() (uart_ingress path, via
 *          LatestData/snap — confirmed to already carry per-slot
 *          event_id thanks to state_registry.c's whole-struct slot
 *          copies, since ReedSlotData/PirSlotData/TempSlotData are the
 *          same types in both SensorData and LatestData) and
 *          uart_update_frame() (pipe_ingress path, via p_raw_frame /
 *          SensorData directly) now emit one EID_<TYPE><slot>: line per
 *          active device into the outbound UART bundle, plus one
 *          "[UART] ... device=<TYPE> slot=<n> eid=..." log line per
 *          device — mirroring the convention already established at
 *          [PARSE], [DISPATCH], and [SHM].
 *
 *          UartMsg.event_id / has_event_id are no longer set by either
 *          function. The doorbell event_id is unaffected by this change
 *          — it continues to ride only in the "[CONTROLLER] -> [UART]
 *          send event_id=..." log line and the DOORBELL: payload line,
 *          as decided separately (doorbell and sensor events are
 *          independent streams, not one merged identity).
 *
 *          CAM has no event_id at any layer today (CamSlotData carries
 *          only age_s/online — confirmed via include/sensor_types.h).
 *          No EID_CAM* lines are emitted. Adding CAM event tracing is a
 *          separate, larger task starting at cam_trigger.c on the
 *          ESP32 side and is out of scope for this change.
 *
 * \note    Wire protocol split — Phase 4B (2026-06-20):
 *          EID_* lines in emit_device_event_ids() are now gated on
 *          event_id != 0 in addition to the active/state check. The hub
 *          delta gate (send_to_bb() / tcp_manager.c) guarantees that
 *          event_id is non-zero in events[] only on true state
 *          transitions; per-slot event_id fields in SensorData are zero
 *          on telemetry-only ticks. Forwarding EID_PIR1:0 etc. to the
 *          STM32 on every heartbeat tick would be noise — zero-on-tick
 *          is expected and suppressed. LOCK and LIGHT already carried
 *          equivalent guards (if (0 != lock_eid)) and are unchanged.
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
#define UART_STATE_UNKNOWN      0xFF    /**< sentinel for unknown reed state  */

/* g_uart_frame_sem and g_uart_ring are defined in data_controller.c
 * and declared extern in uart_controller.h — no definition here.     */

/******************************************************************************
 * \brief Emit one EID_<TYPE><slot>: line per active device into msg.buf,
 *        and one matching [UART] ... device=<TYPE> log line per device.
 *
 * \param p_msg       - UartMsg being built. pos is read and updated.
 * \param p_pos       - In/out cursor into p_msg->buf.
 * \param p_r_state   - Reed state array, UART_STATE_UNKNOWN = inactive slot.
 * \param p_r_eid     - Reed event_id array, indexed the same as p_r_state.
 * \param reed_count  - Number of reed slots to consider.
 * \param p_pactive   - PIR active flags.
 * \param p_p_eid     - PIR event_id array.
 * \param pir_count   - Number of PIR slots to consider.
 * \param p_tactive   - Temp active flags.
 * \param p_t_eid     - Temp event_id array.
 * \param temp_count  - Number of temp slots to consider.
 * \param lock_eid    - Lock event_id, 0 = none this frame.
 * \param light_eid   - Light event_id, 0 = none this frame.
 *
 * \details Shared by both push paths so the wire format and log format
 *          stay identical regardless of which ingress triggered the
 *          push. See file header note "Per-device event_id tracing,
 *          full pipeline (2026-06-19)".
 *
 *          EID_* lines are suppressed when event_id == 0 — see file
 *          header note "Wire protocol split — Phase 4B (2026-06-20)".
 ******************************************************************************/
static void emit_device_event_ids(UartMsg *p_msg, int *p_pos,
                                   const uint8_t  *p_r_state,
                                   const uint64_t *p_r_eid,
                                   int             reed_count,
                                   const uint8_t  *p_pactive,
                                   const uint64_t *p_p_eid,
                                   int             pir_count,
                                   const uint8_t  *p_tactive,
                                   const uint64_t *p_t_eid,
                                   int             temp_count,
                                   uint64_t        lock_eid,
                                   uint64_t        light_eid)
{
   int pos = *p_pos;
   int i   = 0;

   for (i = 0; i < pir_count; i++)
   {
      if (!p_pactive[i] || (0 == p_p_eid[i])) { continue; }
      pos += snprintf(p_msg->buf + pos, sizeof(p_msg->buf) - pos,
                      "EID_PIR%d:%llu\n",
                      i + 1, (unsigned long long)p_p_eid[i]);
      LOG("[UART] transport=ttyS1 write dst=blackpill_lcd device=PIR slot=%d eid=%llu",
          i + 1, (unsigned long long)p_p_eid[i]);
   }

   for (i = 0; i < reed_count; i++)
   {
      if ((UART_STATE_UNKNOWN == p_r_state[i]) || (0 == p_r_eid[i])) { continue; }
      pos += snprintf(p_msg->buf + pos, sizeof(p_msg->buf) - pos,
                      "EID_REED%d:%llu\n",
                      i + 1, (unsigned long long)p_r_eid[i]);
      LOG("[UART] transport=ttyS1 write dst=blackpill_lcd device=REED slot=%d eid=%llu",
          i + 1, (unsigned long long)p_r_eid[i]);
   }

   for (i = 0; i < temp_count; i++)
   {
      if (!p_tactive[i] || (0 == p_t_eid[i])) { continue; }
      pos += snprintf(p_msg->buf + pos, sizeof(p_msg->buf) - pos,
                      "EID_TEMP%d:%llu\n",
                      i + 1, (unsigned long long)p_t_eid[i]);
      LOG("[UART] transport=ttyS1 write dst=blackpill_lcd device=TEMP slot=%d eid=%llu",
          i + 1, (unsigned long long)p_t_eid[i]);
   }

   if (0 != lock_eid)
   {
      pos += snprintf(p_msg->buf + pos, sizeof(p_msg->buf) - pos,
                      "EID_LOCK:%llu\n", (unsigned long long)lock_eid);
      LOG("[UART] transport=ttyS1 write dst=blackpill_lcd device=LOCK eid=%llu",
          (unsigned long long)lock_eid);
   }

   if (0 != light_eid)
   {
      pos += snprintf(p_msg->buf + pos, sizeof(p_msg->buf) - pos,
                      "EID_LIGHT:%llu\n", (unsigned long long)light_eid);
      LOG("[UART] transport=ttyS1 write dst=blackpill_lcd device=LIGHT eid=%llu",
          (unsigned long long)light_eid);
   }

   /* CAM intentionally omitted — CamSlotData carries no event_id today.
    * See file header note. */

   *p_pos = pos;
}

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
 *          Per-device EID_* lines (PIR/REED/TEMP/LOCK/LIGHT) are emitted
 *          via emit_device_event_ids() — see file header note. The
 *          envelope no longer carries a single event_id; the per-device
 *          lines are the only event identity on the wire for this path.
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
                            const uint64_t *p_r_eid,
                            int             pir_count,
                            const uint32_t *p_pc,
                            const int8_t   *p_pb,
                            const uint16_t *p_pa,
                            const uint8_t  *p_pactive,
                            const int      *p_pocc,
                            const uint64_t *p_p_eid,
                            int             temp_count,
                            const int16_t  *p_tc,
                            const int8_t   *p_tb,
                            const uint16_t *p_ta,
                            const uint8_t  *p_tactive,
                            const uint64_t *p_t_eid,
                            int             doorbell_pressed,
                            int             doorbell_device_id,
                            int             db_inference_valid,
                            uint64_t        db_event_id,
                            uint8_t         db_person,
                            uint8_t         db_conf_pct,
                            const char     *db_asset,
                            uint64_t        lock_eid,
                            uint64_t        light_eid)
{
   UartMsg msg = {0};
   int     pos = 0;
   int     i   = 0;

   pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                   "STATE:%d,%d,%d,%d,%d,%d,%d,%d\n",
                   (int)temp, motion, lgt, lck,
                   age_pir, age_lgt, age_lck, reed_count);

   if (0 <= batt_pir)
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "PIR:%d,%d\n", motion, batt_pir);
   }
   else
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "PIR:%d\n", motion);
   }

   pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                   "PIR_COUNT:%d\n", pir_count);

   for (i = 0; i < pir_count; i++)
   {
      if (!p_pactive[i]) { continue; }
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "PIR%d:%u,%d,%d\n",
                      i + 1, (unsigned)p_pc[i], p_pb[i], p_pa[i]);
   }

   for (i = 0; i < pir_count; i++)
   {
      if (!p_pactive[i]) { continue; }
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "OCC%d:%d\n", i + 1, p_pocc[i]);
   }

   pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                   "REED_COUNT:%d\n", reed_count);

   for (i = 0; i < reed_count; i++)
   {
      if (UART_STATE_UNKNOWN == p_r_state[i]) { continue; }
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "DR%d:%d,%d,%d\n",
                      i + 1, p_r_state[i], p_r_batt[i], p_r_age[i]);
   }

   pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos, "LGT:%d,%d\n", lgt, age_lgt);

   if (0 <= batt_lck)
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "LCK:%d,%d,%d\n", lck, batt_lck, age_lck);
   }
   else
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos, "LCK:%d,-1,%d\n", lck, age_lck);
   }

   if (batt_motor > 0)
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "MTR:%d,%d\n", motor_online, batt_motor);
   }
   else
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "MTR:%d\n", motor_online);
   }

   pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                   "TEMP_COUNT:%d\n", temp_count);

   for (i = 0; i < temp_count; i++)
   {
      if (!p_tactive[i]) { continue; }
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "TEMP%d:%d,%d,%d\n",
                      i + 1, p_tc[i], p_tb[i], p_ta[i]);
   }

   if (doorbell_pressed && db_inference_valid)
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
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
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "DOORBELL:%d,%d\n",
                      doorbell_pressed, doorbell_device_id);
   }

   /* Per-device event_id lines — see emit_device_event_ids() and file
    * header note "Per-device event_id tracing, full pipeline
    * (2026-06-19)". Replaces the old single msg.event_id envelope
    * field, which is no longer set. Zero event_ids suppressed per
    * "Wire protocol split — Phase 4B (2026-06-20)". */
   emit_device_event_ids(&msg, &pos,
                          p_r_state, p_r_eid, reed_count,
                          p_pactive, p_p_eid, pir_count,
                          p_tactive, p_t_eid, temp_count,
                          lock_eid, light_eid);

   msg.len = pos;

   uart_push_msg(&msg);
}

/******************************************************************************
 * \brief Thread — sends sensor state to STM32 BlackPill, driven by
 *        g_uart_frame_sem.
 *
 * \param p_arg - Unused thread argument.
 *
 * \return void*
 *
 * \details Blocks on g_uart_frame_sem with UART_PUSH_INTERVAL_SEC watchdog
 *          ceiling. Wakes immediately when uart_parse_line() posts a frame
 *          from the STM32 BlackPill (heartbeat or command), drains all
 *          pending frames from g_uart_ring, then reads snapshot and SHM
 *          to build and send the full STM32 protocol payload.
 *
 *          SHM mutex is held only for doorbell_pressed consume-and-clear.
 *          All sensor state (valid, temp, motion, lgt, lck, slots) comes
 *          from get_snapshot() via state_registry.c — see file header note
 *          "SHM read path (dst=blackpill_lcd)".
 *
 *          Per-device event_id values come from snap (LatestData),
 *          populated via state_registry.c's whole-struct slot copies —
 *          see file header note "Per-device event_id tracing, full
 *          pipeline (2026-06-19)". shm_data->event_id is no longer read;
 *          that field no longer exists upstream.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void *uart_push_thread(void *p_arg)
{
   struct LatestData snap        = {0};
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

   int      reed_count         = 0;
   int      pir_count          = 0;
   int      temp_count         = 0;

   uint8_t  r_state[MAX_REEDS];
   int8_t   r_batt[MAX_REEDS];
   uint16_t r_age[MAX_REEDS];
   uint64_t r_eid[MAX_REEDS];

   uint32_t p_count[MAX_PIRS];
   int8_t   p_batt[MAX_PIRS];
   uint16_t p_age[MAX_PIRS];
   uint8_t  p_active[MAX_PIRS];
   int      p_occ[MAX_PIRS];
   uint64_t p_eid[MAX_PIRS];

   int16_t  t_decidegc[MAX_TEMPS];
   int8_t   t_batt[MAX_TEMPS];
   uint16_t t_age[MAX_TEMPS];
   uint8_t  t_active[MAX_TEMPS];
   uint64_t t_eid[MAX_TEMPS];

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

      if (!uart_transport_is_open()) { continue; }

      /* Rate-limit outbound bundles to 2 per second max.             */
      {
         static struct timespec s_last;
         struct timespec        now;
         clock_gettime(CLOCK_MONOTONIC, &now);
         long elapsed_ms = (now.tv_sec  - s_last.tv_sec)  * 1000L
                         + (now.tv_nsec - s_last.tv_nsec) / 1000000L;
         if (elapsed_ms < 500L) { continue; }
         s_last = now;
      }

      frames_ready = 0;
      if (uart_ring_pop(&frame)) { frames_ready = 1; }
      while (sem_trywait(&g_uart_frame_sem) == 0)
      {
         uart_ring_pop(&frame);
         frames_ready++;
      }

      LOG("[PUSH] Woke: %d UART frame(s) in burst", frames_ready);

      /* Snapshot first — all sensor state comes from state_registry.
       * SHM mutex below is held only for doorbell consume-and-clear.  */
      get_snapshot(&snap);

      if (!snap.valid)
      {
         LOG("[PUSH] No valid data yet, skipping");
         continue;
      }

      db_inference_valid = 0;
      db_event_id        = 0;
      db_person          = 0;
      db_conf_pct        = 0;
      db_asset[0]        = '\0';
      doorbell_pressed   = 0;
      doorbell_device_id = 0;

      /* Consume-and-clear doorbell_pressed — must stay under SHM mutex.
       * This is the only remaining SHM read in this path.             */
      pthread_mutex_lock(&shm_data->shm_mutex);
      doorbell_pressed   = shm_data->doorbell_pressed;
      doorbell_device_id = shm_data->doorbell_device_id;
      if (0 != doorbell_pressed) { shm_data->doorbell_pressed = 0; }
      pthread_mutex_unlock(&shm_data->shm_mutex);

      LOG("[SHM] transport=sensor_shm read dst=blackpill_lcd");

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
            doorbell_pending_mark(doorbell_device_id);
         }
      }

      doorbell_inject_pending(&doorbell_pressed, &doorbell_device_id,
                              &db_inference_valid, &db_event_id,
                              &db_person, &db_conf_pct,
                              db_asset, DOORBELL_ASSET_LEN);

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
         r_eid[i]  = snap.reed_slots[i].event_id;
      }

      pir_count = 0;
      for (i = 0; i < MAX_PIRS; i++)
      {
         p_count[i]  = snap.pir_slots[i].count;
         p_batt[i]   = snap.pir_slots[i].batt;
         p_age[i]    = snap.pir_slots[i].age;
         p_active[i] = snap.pir_slots[i].active;
         p_occ[i]    = snap.pir_slots[i].occupied;
         p_eid[i]    = snap.pir_slots[i].event_id;
         if (snap.pir_slots[i].active) { pir_count = snap.pir_count; }
      }

      temp_count = 0;
      for (i = 0; i < MAX_TEMPS; i++)
      {
         t_decidegc[i] = snap.temp_slots[i].temp_decidegc;
         t_batt[i]     = snap.temp_slots[i].batt;
         t_age[i]      = snap.temp_slots[i].age;
         t_active[i]   = snap.temp_slots[i].active;
         t_eid[i]      = snap.temp_slots[i].event_id;
         if (snap.temp_slots[i].active) { temp_count = snap.temp_count; }
      }

      build_and_push(snap.avg_temp, snap.motion_count,
                     snap.light_state, snap.lock_state,
                     snap.age_pir, snap.age_lgt, snap.age_lck,
                     snap.batt_pir, snap.batt_lck, snap.batt_motor,
                     reed_count, snap.motor_online,
                     r_state, r_batt, r_age, r_eid,
                     pir_count, p_count, p_batt, p_age, p_active, p_occ, p_eid,
                     temp_count, t_decidegc, t_batt, t_age, t_active, t_eid,
                     doorbell_pressed, doorbell_device_id,
                     db_inference_valid, db_event_id, db_person, db_conf_pct,
                     db_asset,
                     snap.lock_event_id, snap.light_event_id);
   }

   LOG("[PUSH] Push thread exiting");
   return NULL;
}

/******************************************************************************
 * \brief Synchronous outbound push to STM32 BlackPill from pipe ingress path.
 *
 * \param p_snapshot  - Frozen read model from get_snapshot(). Not modified.
 * \param p_raw_frame - Raw wire frame from sensor pipe. Used for
 *                      doorbell_slots[] (DB<n> per-cam liveness lines),
 *                      cam_slots[] (CAM<n> liveness lines), and
 *                      per-device event_id (PIR/reed/temp/lock/light).
 *
 * \return void
 *
 * \details Called from sensor_frame_dispatch() on every pipe ingress frame.
 *          Reads doorbell state from SHM under shm_mutex (consume-and-clear
 *          only). cam_online[] is read directly from
 *          p_raw_frame->cam_slots[i].online — confirmed equivalent to
 *          shm_data->cam_online[i] (shm_updater.c line 143 is a direct
 *          assignment with no transformation). SHM mutex is no longer held
 *          around the CAM loop — see file header note "SHM read path".
 *
 *          Per-device event_id values come directly from p_raw_frame
 *          (SensorData) — see file header note "Per-device event_id
 *          tracing, full pipeline (2026-06-19)". shm_data->event_id is
 *          no longer read; that field no longer exists upstream.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_update_frame(const struct LatestData *p_snapshot,
                       const struct SensorData *p_raw_frame)
{
   UartMsg  msg                    = {0};
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

   /* Rate-limit to 2 per second max.                                 */
   {
      static struct timespec s_last;
      struct timespec        now;
      clock_gettime(CLOCK_MONOTONIC, &now);
      long elapsed_ms = (now.tv_sec  - s_last.tv_sec)  * 1000L
                      + (now.tv_nsec - s_last.tv_nsec) / 1000000L;
      if (elapsed_ms < 500L) { return; }
      s_last = now;
   }

   if (!p_snapshot->valid)        { return; }
   if (!uart_transport_is_open()) { return; }

   db_asset[0] = '\0';

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (p_snapshot->reed_slots[i].active) { reed_count = i + 1; }
   }

   pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
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
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "PIR:%d,%d\n",
                      p_snapshot->motion_count, p_snapshot->batt_pir);
   }
   else
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "PIR:%d\n", p_snapshot->motion_count);
   }

   pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                   "PIR_COUNT:%d\n", p_snapshot->pir_count);

   for (i = 0; i < MAX_PIRS; i++)
   {
      if (!p_snapshot->pir_slots[i].active) { continue; }
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "PIR%d:%u,%d,%d\n",
                      i + 1,
                      (unsigned)p_snapshot->pir_slots[i].count,
                      p_snapshot->pir_slots[i].batt,
                      p_snapshot->pir_slots[i].age);
   }

   for (i = 0; i < MAX_PIRS; i++)
   {
      if (!p_snapshot->pir_slots[i].active) { continue; }
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "OCC%d:%d\n", i + 1,
                      p_snapshot->pir_slots[i].occupied);
   }

   pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                   "REED_COUNT:%d\n", reed_count);

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (!p_snapshot->reed_slots[i].active) { continue; }
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "DR%d:%d,%d,%d\n",
                      i + 1,
                      p_snapshot->reed_slots[i].state,
                      p_snapshot->reed_slots[i].batt,
                      p_snapshot->reed_slots[i].age);
   }

   pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                   "LGT:%d,%d\n", p_snapshot->light_state, p_snapshot->age_lgt);

   if (p_snapshot->batt_lck >= 0)
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "LCK:%d,%d,%d\n",
                      p_snapshot->lock_state, p_snapshot->batt_lck, p_snapshot->age_lck);
   }
   else
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "LCK:%d,-1,%d\n", p_snapshot->lock_state, p_snapshot->age_lck);
   }

   if (p_snapshot->batt_motor > 0)
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "MTR:%d,%d\n",
                      p_snapshot->motor_online, p_snapshot->batt_motor);
   }
   else
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "MTR:%d\n", p_snapshot->motor_online);
   }

   pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                   "TEMP_COUNT:%d\n", p_snapshot->temp_count);

   for (i = 0; i < MAX_TEMPS; i++)
   {
      if (!p_snapshot->temp_slots[i].active) { continue; }
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "TEMP%d:%d,%d,%d\n",
                      i + 1,
                      p_snapshot->temp_slots[i].temp_decidegc,
                      p_snapshot->temp_slots[i].batt,
                      p_snapshot->temp_slots[i].age);
   }


   doorbell_pending_check();

   /* Consume-and-clear doorbell_pressed — must stay under SHM mutex.
    * This is the only remaining SHM read in this path.               */
   pthread_mutex_lock(&shm_data->shm_mutex);
   doorbell_pressed   = shm_data->doorbell_pressed;
   doorbell_device_id = shm_data->doorbell_device_id;
   if (doorbell_pressed) { shm_data->doorbell_pressed = 0; }
   pthread_mutex_unlock(&shm_data->shm_mutex);

   LOG("[SHM] transport=sensor_shm read dst=blackpill_lcd");

   if (0 != doorbell_pressed)
   {
      db_inference_valid = doorbell_result_reader_poll(
                              &db_event_id, &db_person, &db_conf_pct,
                              db_asset, sizeof(db_asset));

      if (!db_inference_valid)
      {
         LOG("[CONTROLLER] waiting_for_result device_id=%d", doorbell_device_id);
         doorbell_pending_mark(doorbell_device_id);
      }
   }

   doorbell_inject_pending(&doorbell_pressed, &doorbell_device_id,
                           &db_inference_valid, &db_event_id,
                           &db_person, &db_conf_pct,
                           db_asset, DOORBELL_ASSET_LEN);

   if (doorbell_pressed && db_inference_valid)
   {
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
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
      pos += snprintf(msg.buf + pos, sizeof(msg.buf) - pos,
                      "DOORBELL:%d,%d\n",
                      doorbell_pressed, doorbell_device_id);
   }


   /* Per-device event_id lines, sourced directly from p_raw_frame
    * (SensorData) — see emit_device_event_ids() and file header note
    * "Per-device event_id tracing, full pipeline (2026-06-19)".
    * Replaces the old single msg.event_id envelope field, which is no
    * longer set. Zero event_ids suppressed per "Wire protocol split —
    * Phase 4B (2026-06-20)". Doorbell event_id is unaffected — it
    * continues to ride only in the [CONTROLLER] -> [UART] send log
    * line above and the DOORBELL: payload line, not as an EID_* line,
    * since it is an independent event stream from the sensor devices
    * below.                                                            */
   if (NULL != p_raw_frame)
   {
      uint64_t r_eid[MAX_REEDS];
      uint64_t p_eid[MAX_PIRS];
      uint64_t t_eid[MAX_TEMPS];
      uint8_t  r_state[MAX_REEDS];
      uint8_t  p_active[MAX_PIRS];
      uint8_t  t_active[MAX_TEMPS];

      for (i = 0; i < MAX_REEDS; i++)
      {
         r_state[i] = p_raw_frame->reed_slots[i].active
                      ? p_raw_frame->reed_slots[i].state
                      : UART_STATE_UNKNOWN;
         r_eid[i]   = p_raw_frame->reed_slots[i].event_id;
      }

      for (i = 0; i < MAX_PIRS; i++)
      {
         p_active[i] = p_raw_frame->pir_slots[i].active;
         p_eid[i]    = p_raw_frame->pir_slots[i].event_id;
      }

      for (i = 0; i < MAX_TEMPS; i++)
      {
         t_active[i] = p_raw_frame->temp_slots[i].active;
         t_eid[i]    = p_raw_frame->temp_slots[i].event_id;
      }

      emit_device_event_ids(&msg, &pos,
                             r_state, r_eid, MAX_REEDS,
                             p_active, p_eid, MAX_PIRS,
                             t_active, t_eid, MAX_TEMPS,
                             p_raw_frame->lock_event_id,
                             p_raw_frame->light_event_id);
   }

   msg.len = pos;

   uart_push_msg(&msg);
}
