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
 *          [SHM] read  dst=thermostat_lcd — thermostat_lcd reading temp
 *                                           and motor state for HD44780
 *
 *          SHM architecture — two writers, three readers:
 *
 *          Writers:
 *            pipe_ingress:   sensor_frame_dispatch() → shm_update_frame()
 *                            → handle_get_latest(p_snapshot, "pipe_ingress")
 *            uart_ingress:   uart_parse_line() → handle_get_latest()
 *                            → handle_get_latest(p_snapshot, "uart_ingress")
 *
 *          Readers:
 *            thermostat_lcd: thermostat_lcd.c display update loop.
 *                            Reads current_temp, batt_motor,
 *                            device_online[3], data_valid, sequence.
 *
 *            inference.py:   PIR-triggered inference daemon (Yocto
 *                            recipes-apps/inference). Reads
 *                            current_motion at a fixed struct offset
 *                            to detect PIR motion increments. Does NOT
 *                            use /doorbell_result — that segment is
 *                            exclusively the doorbell path.
 *
 *            uart_controller: doorbell_pressed consume-and-clear ONLY.
 *                            All sensor state now comes from
 *                            get_snapshot() via state_registry.c.
 *                            cam_online[] read directly from p_raw_frame.
 *                            See uart_controller.c header note "SHM read
 *                            path (dst=blackpill_lcd)" (2026-06-19).
 *
 * \note    Structured event tracing (2026-06-16):
 *          handle_get_latest() previously projected a single rolled-up
 *          event_id from p_snapshot into shm_data->event_id under the
 *          shm_mutex. That field is no longer written — see note below.
 *
 * \note    Per-device event_id logging (2026-06-19):
 *          The single rolled-up "event_id=" on the [SHM] write line above
 *          was misleading — it silently picked lock_event_id (falling
 *          back to light_event_id) to represent the *entire* write, while
 *          PIR/reed/temp event IDs — which were equally present in
 *          p_data at this point — were dropped without being logged at
 *          all. A real capture showed event_id=52266 on the SHM write
 *          line exactly matching lock_eid=52266 from the [DISPATCH] line
 *          one row above it, while that same [DISPATCH] line also showed
 *          PIR slot=1 eid=45512, PIR slot=2 eid=45342, and Temp slot=1
 *          eid=52257 — none of which made it into the SHM log.
 *
 *          shm_update_frame() now logs every active device's event_id
 *          individually (device=PIR/REED/TEMP/LOCK/LIGHT), each on its
 *          own [SHM] line, sourced from p_data — which was already being
 *          passed into this function and already carries every per-slot
 *          event_id intact (confirmed via pipe_writer.c's raw struct
 *          write, pipe_reader.c's raw struct read, and sensor_dispatch.c's
 *          [DISPATCH] log, all of which preserve it without loss). No new
 *          plumbing was required — only this function was discarding data
 *          it already had in scope.
 *
 *          The scalar "event_id=" and "seq=" fields are removed from the
 *          [SHM] write log line entirely. event_id= because there is no
 *          single event identity for a multi-device snapshot write — see
 *          above. seq= (shm_data->sequence) because per-device event_id
 *          tracking now provides finer-grained staleness detection than
 *          a single SHM-wide write counter did; shm_data->sequence still
 *          increments internally for any other internal use, it is just
 *          no longer printed here. handle_get_latest() no longer computes
 *          or assigns shm_data->event_id.
 *
 * \note    Wire protocol split — Phase 4B (2026-06-20):
 *          event_id fields in per-slot objects are zero on telemetry-only
 *          ticks — the hub delta gate (send_to_bb() / tcp_manager.c)
 *          guarantees non-zero only when a true state transition was
 *          emitted in events[] that tick. Zero-guards added to the
 *          PIR/REED/TEMP loops in log_shm_device_event_ids() to suppress
 *          no-event ticks from the SHM event log. LOCK and LIGHT already
 *          carried equivalent guards (if (0 != p_data->lock_event_id))
 *          and are unchanged.
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
static void update_shm_reed_liveness(const struct SensorData *p_data)
{
   int i = 0;

   pthread_mutex_lock(&shm_data->shm_mutex);

   for (i = 0; i < MAX_REEDS; i++)
   {
      shm_data->reed_age_s[i]  = p_data->reed_slots[i].age;
      shm_data->reed_online[i] = p_data->reed_slots[i].active;
   }

   pthread_mutex_unlock(&shm_data->shm_mutex);
}
static void update_shm_pir_liveness(const struct SensorData *p_data)
{
   int i = 0;

   pthread_mutex_lock(&shm_data->shm_mutex);

   for (i = 0; i < MAX_PIRS; i++)
   {
      shm_data->pir_age_s[i]  = p_data->pir_slots[i].age;
      shm_data->pir_online[i] = p_data->pir_slots[i].active;
   }

   pthread_mutex_unlock(&shm_data->shm_mutex);
}

/******************************************************************************
 * \brief Log each active device's event_id individually from p_data.
 *
 * \param p_data - Full SensorData wire frame (not the LatestData
 *                 snapshot) — this is the only struct that carries
 *                 per-slot PIR/reed/temp event IDs.
 *
 * \details Replaces the old single rolled-up event_id= on the [SHM]
 *          write line. See file-header note "Per-device event_id
 *          logging (2026-06-19)" for why.
 *
 *          Zero-guards on PIR/REED/TEMP suppress telemetry-only ticks
 *          where no event was emitted by the hub — see file-header note
 *          "Wire protocol split — Phase 4B (2026-06-20)".
 ******************************************************************************/
static void log_shm_device_event_ids(const struct SensorData *p_data)
{
   int i = 0;

   for (i = 0; i < MAX_PIRS; i++)
   {
      if (p_data->pir_slots[i].active && (0 != p_data->pir_slots[i].event_id))
      {
         LOG("[SHM] transport=sensor_shm write src=pipe_ingress device=PIR slot=%d eid=%llu",
             i + 1, (unsigned long long)p_data->pir_slots[i].event_id);
      }
   }

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (p_data->reed_slots[i].active && (0 != p_data->reed_slots[i].event_id))
      {
         LOG("[SHM] transport=sensor_shm write src=pipe_ingress device=REED slot=%d eid=%llu",
             i + 1, (unsigned long long)p_data->reed_slots[i].event_id);
      }
   }

   for (i = 0; i < MAX_TEMPS; i++)
   {
      if (p_data->temp_slots[i].active && (0 != p_data->temp_slots[i].event_id))
      {
         LOG("[SHM] transport=sensor_shm write src=pipe_ingress device=TEMP slot=%d eid=%llu",
             i + 1, (unsigned long long)p_data->temp_slots[i].event_id);
      }
   }

   if (0 != p_data->lock_event_id)
   {
      LOG("[SHM] transport=sensor_shm write src=pipe_ingress device=LOCK eid=%llu",
          (unsigned long long)p_data->lock_event_id);
   }

   if (0 != p_data->light_event_id)
   {
      LOG("[SHM] transport=sensor_shm write src=pipe_ingress device=LIGHT eid=%llu",
          (unsigned long long)p_data->light_event_id);
   }
}

void handle_get_latest(const struct LatestData *p_snapshot, const char *p_src)
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

   /* Log from p_snapshot — not from shm_data — so the line reflects
    * exactly what was written even if another thread writes immediately
    * after the unlock. */
   LOG("[SHM] transport=sensor_shm write src=%s temp=%.1fC motion=%d valid=%d",
       p_src,
       p_snapshot->avg_temp,
       p_snapshot->motion_count,
       p_snapshot->valid);
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
   log_shm_device_event_ids(p_data);
   update_shm_rooms(p_data);
   update_shm_doorbell_liveness(p_data);
   update_shm_cam_liveness(p_data);
   update_shm_reed_liveness(p_data);
   update_shm_pir_liveness(p_data);
}
