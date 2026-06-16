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
 *          [SHM] update, get_latest, get_device_status, get_room_status,
 *          doorbell_press
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

void handle_get_latest(const struct LatestData *p_snapshot)
{
   pthread_mutex_lock(&shm_data->shm_mutex);

   shm_data->current_temp      = p_snapshot->avg_temp;
   shm_data->current_motion    = p_snapshot->motion_count;
   shm_data->current_light     = p_snapshot->light_state;
   shm_data->current_lock      = p_snapshot->lock_state;
   shm_data->batt_motor        = p_snapshot->batt_motor;
   shm_data->current_timestamp = p_snapshot->timestamp;
   shm_data->data_valid        = p_snapshot->valid;
   shm_data->sequence++;

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

   pthread_mutex_unlock(&shm_data->shm_mutex);

   LOG("[SHM] update temp=%.1fC motion=%d valid=%d seq=%u",
       shm_data->current_temp,
       shm_data->current_motion,
       shm_data->data_valid,
       shm_data->sequence);
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
   handle_get_latest(p_snapshot);
   update_shm_rooms(p_data);
   update_shm_doorbell_liveness(p_data);
   update_shm_cam_liveness(p_data);
}
