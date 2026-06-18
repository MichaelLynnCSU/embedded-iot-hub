/******************************************************************************
 * \file shm_updater.c
 * \brief Shared memory projection — consumes frozen snapshot, writes shm.
 *
 * \note    Doorbell liveness (2026-06-09):
 *          shm_update_frame() now projects p_data->doorbell_slots[] into
 *          shm_data->doorbell_age_s[] and shm_data->doorbell_online[].
 *          Runs under shm_mutex after update_shm_rooms() returns.
 *          p_data is used directly (not via snapshot) because doorbell
 *          liveness is not part of LatestData — it lives only in the
 *          raw SensorData wire frame.
 *
 * \note    Logging taxonomy (2026-06-16):
 *          [SHM] write src=pipe_ingress  — sensor frame from named pipe
 *          [SHM] write src=uart_ingress  — frame from STM32 BlackPill UART
 *          [SHM] read  dst=blackpill_lcd — uart_push_thread reading to build
 *                                          UART bundle for STM32 BlackPill
 *          [SHM] read  dst=thermostat_lcd — motor_lcd reading temp and motor
 *                                           state for HD44780 display
 *          [SHM] get_device_status
 *          [SHM] get_room_status
 *          [SHM] doorbell_press
 *
 * \note    Structured event tracing (2026-06-16):
 *          handle_get_latest() projects event_id from p_snapshot into
 *          shm_data->event_id under the shm_mutex, then logs it after
 *          the lock is released using p_snapshot (stable, not re-read
 *          from shm).
 *
 *          SHM has two writers and two readers:
 *
 *            [SHM] transport=sensor_shm event_id=M write src=pipe_ingress
 *            [SHM] transport=sensor_shm event_id=M write src=uart_ingress
 *            [SHM] transport=sensor_shm event_id=M read  dst=blackpill_lcd
 *            [SHM] transport=sensor_shm event_id=M read  dst=thermostat_lcd
 *
 *          pipe_ingress: sensor data from ESP32 via named pipe
 *                        sensor_frame_dispatch() → shm_update_frame()
 *                        → handle_get_latest()
 *
 *          uart_ingress: heartbeat and command frames from STM32 BlackPill
 *                        uart_parse_line() → handle_get_latest()
 *
 *          blackpill_lcd: uart_push_thread() reads SHM to build the UART
 *                         bundle pushed to STM32 BlackPill every 5s
 *
 *          thermostat_lcd: motor_lcd reads current_temp and batt_motor
 *                          for the HD44780 motor thermostat display
 *
 *          event_id is the most recent non-zero value from lock_event_id
 *          or light_event_id in the snapshot. PIR/reed/temp event IDs are
 *          per-slot and not scalar — they are not projected here; their
 *          trace ends at [DISPATCH].
 ******************************************************************************/

#include "shm_updater.h"
#include "log.h"
#include "heartbeat.h"
#include "db_manager.h"
#include <string.h>
#include <stdio.h>

static void update_shm_rooms(const struct SensorData *p_data)
{
   int i = 0;

   pthread_mutex_lock(&shm_data->shm_mutex);
   shm_data->room_count = p_data->room_count;

   for (i = 0; (i < p_data->room_count) && (i < MAX_ROOMS); i++)
   {
      shm_data->rooms[i].sensor_id = p_data->rooms[i].sensor_id;
      (void)strncpy(shm_data->rooms[i].room_name,
                    p_data->rooms[i].room_name,
                    ROOM_NAME_SIZE - 1);
      (void)strncpy(shm_data->rooms[i].state,
                    p_data->rooms[i].state,
                    ROOM_STATE_SIZE - 1);
      (void)strncpy(shm_data->rooms[i].location,
                    p_data->rooms[i].location,
                    ROOM_LOC_SIZE - 1);
   }

   pthread_mutex_unlock(&shm_data->shm_mutex);
}

static void update_shm_doorbell_liveness(const struct SensorData *p_data)
{
   int i = 0;

   pthread_mutex_lock(&shm_data->shm_mutex);

   for (i = 0; i < MAX_DOORBELL_CAMS; i++)
   {
      shm_data->doorbell_age_s[i]  = p_data->doorbell_slots[i].age_s;
      shm_data->doorbell_online[i] = p_data->doorbell_slots[i].online;
   }

   pthread_mutex_unlock(&shm_data->shm_mutex);
}

static void update_shm_cam_liveness(const struct SensorData *p_data)
{
   int i = 0;

   pthread_mutex_lock(&shm_data->shm_mutex);

   for (i = 0; i < MAX_CAMS; i++)
   {
      shm_data->cam_age_s[i]  = p_data->cam_slots[i].age_s;
      shm_data->cam_online[i] = p_data->cam_slots[i].online;
   }

   pthread_mutex_unlock(&shm_data->shm_mutex);
}

void handle_get_latest(const struct LatestData *p_snapshot, const char *p_src)
{
   uint32_t seq      = 0;
   uint64_t event_id = 0;

   /* Pick the most recent non-zero event_id from the snapshot.
    * lock and light are the only scalar event IDs that flow through
    * LatestData. PIR/reed/temp are per-slot and end at [DISPATCH]. */
   event_id = p_snapshot->lock_event_id
              ? p_snapshot->lock_event_id
              : p_snapshot->light_event_id;

   pthread_mutex_lock(&shm_data->shm_mutex);

   shm_data->current_temp      = p_snapshot->avg_temp;
   shm_data->current_motion    = p_snapshot->motion_count;
   shm_data->current_light     = p_snapshot->light_state;
   shm_data->current_lock      = p_snapshot->lock_state;
   shm_data->batt_motor        = p_snapshot->batt_motor;
   shm_data->current_timestamp = p_snapshot->timestamp;
   shm_data->data_valid        = p_snapshot->valid;
   shm_data->sequence++;
   shm_data->event_id          = event_id;

   if (p_snapshot->doorbell_pressed)
   {
      shm_data->doorbell_pressed   = 1;
      shm_data->doorbell_device_id = p_snapshot->doorbell_device_id;
      shm_data->doorbell_timestamp = p_snapshot->timestamp;
      LOG("[SHM] doorbell_press device_id=%d",
          p_snapshot->doorbell_device_id);
   }

   shm_data->last_command   = CMD_GET_LATEST;
   shm_data->command_result = 0;

   seq = shm_data->sequence;

   pthread_mutex_unlock(&shm_data->shm_mutex);

   /* Log from p_snapshot — not from shm_data — so the line reflects
    * exactly what was written even if another thread writes immediately
    * after the unlock. */
   LOG("[SHM] transport=sensor_shm event_id=%llu write src=%s "
       "temp=%.1fC motion=%d valid=%d seq=%u",
       (unsigned long long)event_id,
       p_src,
       p_snapshot->avg_temp,
       p_snapshot->motion_count,
       p_snapshot->valid,
       seq);
}

void handle_get_device_status(struct CommandMsg *p_cmd)
{
   (void)p_cmd;

   pthread_mutex_lock(&shm_data->shm_mutex);
   heartbeat_snapshot_online(shm_data->device_online, DEV_COUNT);
   shm_data->last_command   = CMD_GET_DEVICE_STATUS;
   shm_data->command_result = 0;
   shm_data->sequence++;
   pthread_mutex_unlock(&shm_data->shm_mutex);

   LOG("[SHM] get_device_status seq=%u", shm_data->sequence);
}

void handle_get_room_status(struct CommandMsg *p_cmd)
{
   struct RoomStatus rooms[ROOM_BUF_SIZE];
   int               count = 0;
   int               i     = 0;

   (void)p_cmd;
   (void)memset(rooms, 0, sizeof(rooms));

   count = db_query_rooms(rooms, ROOM_BUF_SIZE);

   pthread_mutex_lock(&shm_data->shm_mutex);

   if (0 > count)
   {
      shm_data->command_result = -1;
   }
   else
   {
      for (i = 0; i < count; i++)
      {
         shm_data->rooms[i].sensor_id = rooms[i].sensor_id;
         shm_data->rooms[i].timestamp = rooms[i].timestamp;
         (void)strncpy(shm_data->rooms[i].room_name,
                       rooms[i].room_name,
                       ROOM_NAME_SIZE - 1);
         (void)strncpy(shm_data->rooms[i].state,
                       rooms[i].state,
                       ROOM_STATE_SIZE - 1);
         (void)strncpy(shm_data->rooms[i].location,
                       rooms[i].location,
                       ROOM_LOC_SIZE - 1);
      }

      shm_data->room_count     = count;
      shm_data->last_command   = CMD_GET_ROOM_STATUS;
      shm_data->command_result = 0;
      shm_data->sequence++;
   }

   pthread_mutex_unlock(&shm_data->shm_mutex);

   LOG("[SHM] get_room_status count=%d", count);
}

void shm_update_frame(const struct LatestData *p_snapshot,
                      const struct SensorData *p_data)
{
   handle_get_latest(p_snapshot, "pipe_ingress");
   update_shm_rooms(p_data);
   update_shm_doorbell_liveness(p_data);
   update_shm_cam_liveness(p_data);
}
