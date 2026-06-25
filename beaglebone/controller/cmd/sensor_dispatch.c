/******************************************************************************
 * \file sensor_dispatch.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-09
 *
 * \brief Sensor frame dispatch for BeagleBone controller.
 *
 * \details Owns sensor_frame_dispatch() — the fanout point for every
 *          inbound sensor frame. Receives a validated SensorData struct
 *          from pipe_reader.c and routes it to all downstream subsystems:
 *
 *          1. heartbeat_stamp()    — motor liveness
 *          2. update_snapshot()    — mutate canonical state registry
 *          3. get_snapshot()       — freeze read model
 *          4. shm_update_frame()   — project to shared memory
 *          5. db_persist_frame()   — persist to SQLite
 *          6. uart_update_frame()  — push to STM32 via UART
 *
 *          Single responsibility: fanout only. No pipe I/O, no threading,
 *          no command handling. Testable by passing a stack-allocated
 *          SensorData and asserting each subsystem was called.
 *
 *          Dependency direction:
 *            pipe_reader.c -> sensor_dispatch.c -> shm/db/uart subsystems
 *
 * \note    Logging taxonomy (2026-06-16):
 *          [DISPATCH] receive — one summary line per frame with event IDs.
 *          fanout_complete removed (noise — every frame logged it).
 ******************************************************************************/
#include "config.h"
#include "sensor_types.h"
#include "log.h"
#include "heartbeat.h"
#include "state_registry.h"
#include "shm_updater.h"
#include "db_persist.h"
#include "uart_controller.h"
#include "sensor_dispatch.h"

/******************************************************************************
 * \brief Receive and process one complete sensor data frame.
 *
 * \param p_data - Pointer to validated SensorData frame from pipe_reader.c.
 ******************************************************************************/
void sensor_frame_dispatch(const struct SensorData *p_data)
{
   struct LatestData snapshot;
   int               i = 0;

   /* ---- Summary line ---- */
   LOG("[DISPATCH] seq=%u tmp=%.1f mot=%d occ=%d lgt=%d lck=%d mtr=%d "
       "ages pir=%d lgt=%d lck=%d batts pir=%d%% lck=%d%% mtr=%d%% "
       "pirs=%d lock_eid=%llu light_eid=%llu",
       p_data->frame_seq,
       p_data->avg_temp,
       p_data->motion_count,
       p_data->pir_occupied,
       p_data->light_state,
       p_data->lock_state,
       p_data->motor_online,
       p_data->age_pir,
       p_data->age_lgt,
       p_data->age_lck,
       p_data->batt_pir,
       p_data->batt_lck,
       p_data->batt_motor,
       p_data->pir_count,
       (unsigned long long)p_data->lock_event_id,
       (unsigned long long)p_data->light_event_id);

   /* ---- Per-slot PIR (all slots) ---- */
   for (i = 0; i < MAX_PIRS; i++)
   {
      LOG("[DISPATCH] PIR slot=%d cnt=%u batt=%d age=%d occ=%d eid=%llu",
          i + 1,
          (unsigned)p_data->pir_slots[i].count,
          p_data->pir_slots[i].batt,
          p_data->pir_slots[i].age,
          p_data->pir_slots[i].occupied,
          (unsigned long long)p_data->pir_slots[i].event_id);
   }

   /* ---- Per-slot Reed (all slots) ---- */
   for (i = 0; i < MAX_REEDS; i++)
   {
      LOG("[DISPATCH] Reed slot=%d active=%d st=%d batt=%d age=%d eid=%llu",
          i + 1,
          p_data->reed_slots[i].active,
          p_data->reed_slots[i].state,
          p_data->reed_slots[i].batt,
          p_data->reed_slots[i].age,
          (unsigned long long)p_data->reed_slots[i].event_id);
   }

   /* ---- Per-slot Temp (all slots) ---- */
   for (i = 0; i < MAX_TEMPS; i++)
   {
      LOG("[DISPATCH] Temp slot=%d temp=%d batt=%d age=%d eid=%llu",
          i + 1,
          p_data->temp_slots[i].temp_decidegc,
          p_data->temp_slots[i].batt,
          p_data->temp_slots[i].age,
          (unsigned long long)p_data->temp_slots[i].event_id);
   }

   if (p_data->motor_online)
   {
      heartbeat_stamp(DEV_MOTOR);
   }
   if (p_data->age_pir < 30)
   {
      heartbeat_stamp(DEV_PIR);
   }
   if (p_data->age_lgt < 300)
   {
      heartbeat_stamp(DEV_LIGHT);
   }
   if (p_data->age_lck < 300)
   {
      heartbeat_stamp(DEV_LOCK);
   }

   update_snapshot(p_data);
   get_snapshot(&snapshot);
   shm_update_frame(&snapshot, p_data);
   db_persist_frame(p_data);
   uart_update_frame(&snapshot, p_data);
}
