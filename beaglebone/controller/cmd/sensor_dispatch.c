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
 *          [DISPATCH] receive, heartbeat_stamped, fanout_complete
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
 *
 * \return void
 *
 * \details Stamps motor heartbeat if motor is online, then fans the frame
 *          out to all downstream subsystems via the frozen snapshot pattern.
 *          p_data is passed raw to shm_update_frame(), db_persist_frame(),
 *          and uart_update_frame() for fields not captured in LatestData
 *          (e.g. doorbell_slots[]).
 ******************************************************************************/
void sensor_frame_dispatch(const struct SensorData *p_data)
{
   struct LatestData snapshot;

   LOG("[DISPATCH] receive temp=%.1f motion=%d occ=%d lgt=%d lck=%d mtr=%d "
       "ages pir=%d lgt=%d lck=%d batt pir=%d%% lck=%d%% motor=%d%% pirs=%d",
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
       p_data->pir_count);

   if (p_data->motor_online)
   {
      heartbeat_stamp(DEV_MOTOR);
      LOG("[DISPATCH] heartbeat_stamped dev=motor");
   }

   /* Mutate canonical state registry */
   update_snapshot(p_data);

   /* Freeze read model — atomic struct copy */
   get_snapshot(&snapshot);

   /* Fan out to all subsystems */
   shm_update_frame(&snapshot, p_data);
   db_persist_frame(p_data);
   uart_update_frame(&snapshot, p_data);

   LOG("[DISPATCH] fanout_complete");
}
