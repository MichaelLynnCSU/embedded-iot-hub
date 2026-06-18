/******************************************************************************
 * \file json_parser.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-05-10
 *
 * \brief JSON parsing layer for sensor server.
 *
 * \details Parses complete JSON frames into SensorData structs and
 *          forwards them to the controller via pipe_writer.
 *          Knows nothing about UART transport or buffer management.
 *
 *          Split from sensor_server.c (2026-05-10).
 *
 *          PIR slot array added (2026-05-XX).
 *          Logging improvements (2026-05-21).
 *          Doorbell liveness (2026-06-09).
 *
 *          Log taxonomy (2026-06-15):
 *          All log lines use [PARSE] domain prefix with frame_seq=<n>.
 *
 *          Structured event tracing (2026-06-16):
 *          event_id parsed from per-slot JSON into ReedSlotData,
 *          PirSlotData, TempSlotData. lock_event_id and light_event_id
 *          parsed into SensorData. Slot summary logs condensed to one
 *          line per device type.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <json-c/json.h>
#include "json_parser.h"
#include "pipe_writer.h"
#include "sensor_types.h"
#include "server_log.h"

static uint32_t g_frame_seq = 0;

/******************************************************************************
 * \brief Extract age value from JSON object with range clamping.
 ******************************************************************************/
static uint16_t json_get_age(struct json_object *p_root, const char *p_key)
{
   struct json_object *p_obj = NULL;
   int                 v     = 0;

   if (!json_object_object_get_ex(p_root, p_key, &p_obj)) { return AGE_UNKNOWN; }
   v = json_object_get_int(p_obj);
   if (0 > v)            { return AGE_UNKNOWN; }
   if (v > (int)AGE_MAX) { return (uint16_t)AGE_MAX; }
   return (uint16_t)v;
}

/******************************************************************************
 * \brief Parse reed slots from JSON reeds array into SensorData.
 ******************************************************************************/
static void parse_reed_slots(struct json_object *p_reeds_arr,
                              struct SensorData  *p_data)
{
   int                 n      = 0;
   int                 i      = 0;
   int                 id     = 0;
   int                 slot   = 0;
   int                 age    = 0;
   struct json_object *p_r    = NULL;
   struct json_object *p_jid  = NULL;
   struct json_object *p_jval = NULL;

   n = json_object_array_length(p_reeds_arr);

   for (i = 0; i < n; i++)
   {
      p_r = json_object_array_get_idx(p_reeds_arr, i);
      if (!json_object_object_get_ex(p_r, "id", &p_jid)) { continue; }
      id   = json_object_get_int(p_jid);
      slot = id - 1;
      if ((0 > slot) || (slot >= MAX_REEDS)) { continue; }

      p_data->reed_slots[slot].active = 1;

      if (json_object_object_get_ex(p_r, "batt", &p_jval))
         p_data->reed_slots[slot].batt = (int8_t)json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "age", &p_jval))
      {
         age = json_object_get_int(p_jval);
         p_data->reed_slots[slot].age = (0 > age) ? AGE_UNKNOWN : (uint16_t)age;
      }

      if (json_object_object_get_ex(p_r, "state", &p_jval))
         p_data->reed_slots[slot].state = (uint8_t)json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "offline", &p_jval))
         p_data->reed_slots[slot].offline = (uint8_t)json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "gen", &p_jval))
         p_data->reed_slots[slot].gen = (uint16_t)json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "name", &p_jval))
         (void)strncpy(p_data->reed_slots[slot].name,
                       json_object_get_string(p_jval), REED_NAME_LEN - 1);

      if (json_object_object_get_ex(p_r, "event_id", &p_jval))
         p_data->reed_slots[slot].event_id = (uint64_t)json_object_get_int64(p_jval);
   }
}

/******************************************************************************
 * \brief Parse PIR slots from JSON pirs array into SensorData.
 ******************************************************************************/
static void parse_pir_slots(struct json_object *p_pirs_arr,
                             struct SensorData  *p_data)
{
   int                 n      = 0;
   int                 i      = 0;
   int                 id     = 0;
   int                 slot   = 0;
   int                 age    = 0;
   struct json_object *p_r    = NULL;
   struct json_object *p_jid  = NULL;
   struct json_object *p_jval = NULL;

   n = json_object_array_length(p_pirs_arr);

   for (i = 0; i < n; i++)
   {
      p_r = json_object_array_get_idx(p_pirs_arr, i);
      if (!json_object_object_get_ex(p_r, "id", &p_jid)) { continue; }
      id   = json_object_get_int(p_jid);
      slot = id - 1;
      if ((0 > slot) || (slot >= MAX_PIRS)) { continue; }

      p_data->pir_slots[slot].active = 1;

      if (json_object_object_get_ex(p_r, "count", &p_jval))
         p_data->pir_slots[slot].count = (uint32_t)json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "batt", &p_jval))
         p_data->pir_slots[slot].batt = (int8_t)json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "age", &p_jval))
      {
         age = json_object_get_int(p_jval);
         p_data->pir_slots[slot].age = (0 > age) ? AGE_UNKNOWN : (uint16_t)age;
      }

      if (json_object_object_get_ex(p_r, "occupied", &p_jval))
         p_data->pir_slots[slot].occupied = (int8_t)json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "offline", &p_jval))
         p_data->pir_slots[slot].offline = (uint8_t)json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "event_id", &p_jval))
         p_data->pir_slots[slot].event_id = (uint64_t)json_object_get_int64(p_jval);
   }
}

/******************************************************************************
 * \brief Parse "temps" JSON array into p_data->temp_slots[].
 ******************************************************************************/
static void parse_temp_slots(struct json_object *p_temps_arr,
                              struct SensorData  *p_data)
{
   int                 n      = 0;
   int                 i      = 0;
   int                 id     = 0;
   int                 slot   = 0;
   int                 age    = 0;
   struct json_object *p_r    = NULL;
   struct json_object *p_jid  = NULL;
   struct json_object *p_jval = NULL;

   n = json_object_array_length(p_temps_arr);

   for (i = 0; i < n; i++)
   {
      p_r = json_object_array_get_idx(p_temps_arr, i);
      if (!json_object_object_get_ex(p_r, "id", &p_jid)) { continue; }
      id   = json_object_get_int(p_jid);
      slot = id - 1;
      if ((0 > slot) || (slot >= MAX_TEMPS)) { continue; }

      p_data->temp_slots[slot].active = 1;

      if (json_object_object_get_ex(p_r, "temp", &p_jval))
         p_data->temp_slots[slot].temp_decidegc =
            (int16_t)(json_object_get_int(p_jval) * 10);

      if (json_object_object_get_ex(p_r, "batt", &p_jval))
         p_data->temp_slots[slot].batt = (int8_t)json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "age", &p_jval))
      {
         age = json_object_get_int(p_jval);
         p_data->temp_slots[slot].age = (0 > age) ? AGE_UNKNOWN : (uint16_t)age;
      }

      if (json_object_object_get_ex(p_r, "offline", &p_jval))
         p_data->temp_slots[slot].offline = (uint8_t)json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "name", &p_jval))
      {
         const char *p_name = json_object_get_string(p_jval);
         if (NULL != p_name)
         {
            strncpy(p_data->temp_slots[slot].name, p_name, TEMP_NAME_LEN - 1);
            p_data->temp_slots[slot].name[TEMP_NAME_LEN - 1] = '\0';
         }
      }

      if (json_object_object_get_ex(p_r, "event_id", &p_jval))
         p_data->temp_slots[slot].event_id = (uint64_t)json_object_get_int64(p_jval);
   }
}

/******************************************************************************
 * \brief Parse "doorbells" JSON array into p_data->doorbell_slots[].
 ******************************************************************************/
static void parse_doorbell_slots(struct json_object *p_doorbells_arr,
                                  struct SensorData  *p_data)
{
   int                 n      = 0;
   int                 i      = 0;
   int                 id     = 0;
   int                 age    = 0;
   struct json_object *p_r    = NULL;
   struct json_object *p_jid  = NULL;
   struct json_object *p_jval = NULL;

   n = json_object_array_length(p_doorbells_arr);

   for (i = 0; i < n; i++)
   {
      p_r = json_object_array_get_idx(p_doorbells_arr, i);
      if (!json_object_object_get_ex(p_r, "id", &p_jid)) { continue; }
      id = json_object_get_int(p_jid);
      if ((0 > id) || (id >= MAX_DOORBELL_CAMS)) { continue; }

      if (json_object_object_get_ex(p_r, "age_s", &p_jval))
      {
         age = json_object_get_int(p_jval);
         p_data->doorbell_slots[id].age_s =
            (0 > age) ? AGE_UNKNOWN : (uint16_t)age;
      }

      if (json_object_object_get_ex(p_r, "online", &p_jval))
         p_data->doorbell_slots[id].online = (uint8_t)json_object_get_int(p_jval);
   }
}

/******************************************************************************
 * \brief Parse "cams" JSON array into p_data->cam_slots[].
 ******************************************************************************/
static void parse_cam_slots(struct json_object *p_cams_arr,
                             struct SensorData  *p_data)
{
   int                 n      = 0;
   int                 i      = 0;
   int                 id     = 0;
   int                 age    = 0;
   struct json_object *p_r    = NULL;
   struct json_object *p_jid  = NULL;
   struct json_object *p_jval = NULL;

   n = json_object_array_length(p_cams_arr);

   for (i = 0; i < n; i++)
   {
      p_r = json_object_array_get_idx(p_cams_arr, i);
      if (!json_object_object_get_ex(p_r, "id", &p_jid)) { continue; }
      id = json_object_get_int(p_jid);
      if ((0 > id) || (id >= MAX_CAMS)) { continue; }

      if (json_object_object_get_ex(p_r, "age_s", &p_jval))
      {
         age = json_object_get_int(p_jval);
         p_data->cam_slots[id].age_s = (0 > age) ? AGE_UNKNOWN : (uint16_t)age;
      }

      if (json_object_object_get_ex(p_r, "online", &p_jval))
         p_data->cam_slots[id].online = (uint8_t)json_object_get_int(p_jval);
   }
}

/******************************************************************************
 * \brief Parse room entries from JSON rooms array into SensorData.
 ******************************************************************************/
static void parse_rooms(struct json_object *p_rooms_arr,
                         struct SensorData  *p_data)
{
   int                 n      = 0;
   int                 i      = 0;
   struct json_object *p_r    = NULL;
   struct json_object *p_jval = NULL;

   n = json_object_array_length(p_rooms_arr);

   for (i = 0; (i < n) && (p_data->room_count < MAX_ROOMS); i++)
   {
      p_r = json_object_array_get_idx(p_rooms_arr, i);
      if (!json_object_object_get_ex(p_r, "sensor_id", &p_jval)) { continue; }

      p_data->rooms[p_data->room_count].sensor_id = json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "room", &p_jval))
         (void)strncpy(p_data->rooms[p_data->room_count].room_name,
                       json_object_get_string(p_jval), ROOM_NAME_LEN - 1);

      if (json_object_object_get_ex(p_r, "state", &p_jval))
         (void)strncpy(p_data->rooms[p_data->room_count].state,
                       json_object_get_string(p_jval), ROOM_STATE_LEN - 1);

      if (json_object_object_get_ex(p_r, "location", &p_jval))
         (void)strncpy(p_data->rooms[p_data->room_count].location,
                       json_object_get_string(p_jval), ROOM_LOC_LEN - 1);

      p_data->room_count++;
   }
}

/******************************************************************************
 * \brief Parse a JSON body string into SensorData and write to pipe.
 ******************************************************************************/
void process_json(const char *p_json_body)
{
   struct json_object *p_root    = NULL;
   struct json_object *p_obj     = NULL;
   struct SensorData   data;
   int                 i         = 0;
   uint32_t            frame_seq = 0;

   frame_seq = ++g_frame_seq;

   p_root = json_tokener_parse(p_json_body);
   if (NULL == p_root)
   {
      log_msg("[PARSE] decode_failed frame_seq=%u", frame_seq);
      return;
   }

   (void)memset(&data, 0, sizeof(data));

   data.timestamp  = time(NULL);
   data.age_pir    = AGE_UNKNOWN;
   data.age_lgt    = AGE_UNKNOWN;
   data.age_lck    = AGE_UNKNOWN;
   data.batt_pir   = -1;
   data.batt_lck   = -1;
   data.batt_motor = -1;

   for (i = 0; i < MAX_REEDS; i++)
   {
      data.reed_slots[i].age     = AGE_UNKNOWN;
      data.reed_slots[i].batt    = -1;
      data.reed_slots[i].active  = 0;
      data.reed_slots[i].state   = 0xFF;
      data.reed_slots[i].offline = 0;
      data.reed_slots[i].gen     = 0;
      data.reed_slots[i].event_id = 0;
   }

   for (i = 0; i < MAX_PIRS; i++)
   {
      data.pir_slots[i].age      = AGE_UNKNOWN;
      data.pir_slots[i].batt     = -1;
      data.pir_slots[i].active   = 0;
      data.pir_slots[i].count    = 0;
      data.pir_slots[i].offline  = 0;
      data.pir_slots[i].occupied = 0;
      data.pir_slots[i].event_id = 0;
   }

   for (i = 0; i < MAX_TEMPS; i++)
      data.temp_slots[i].event_id = 0;

   for (i = 0; i < MAX_DOORBELL_CAMS; i++)
   {
      data.doorbell_slots[i].age_s  = AGE_UNKNOWN;
      data.doorbell_slots[i].online = 0;
   }
   data.doorbell_count = MAX_DOORBELL_CAMS;

   for (i = 0; i < MAX_CAMS; i++)
   {
      data.cam_slots[i].age_s  = AGE_UNKNOWN;
      data.cam_slots[i].online = 0;
   }

   data.lock_event_id  = 0;
   data.light_event_id = 0;

   if (json_object_object_get_ex(p_root, "avg_temp",          &p_obj)) data.avg_temp          = json_object_get_double(p_obj);
   if (json_object_object_get_ex(p_root, "motion_count",      &p_obj)) data.motion_count       = json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "light_state",       &p_obj)) data.light_state        = json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "lock_state",        &p_obj)) data.lock_state         = json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "batt_pir",          &p_obj)) data.batt_pir           = (int8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "pir_occupied",      &p_obj)) data.pir_occupied       = (int8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "doorbell_pressed",  &p_obj)) data.doorbell_pressed   = (uint8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "doorbell_device_id",&p_obj)) data.doorbell_device_id = (uint8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "batt_lck",          &p_obj)) data.batt_lck           = (int8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "motor_online",      &p_obj)) data.motor_online       = (uint8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "batt_motor",        &p_obj)) data.batt_motor         = json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "pir_count",         &p_obj)) data.pir_count          = (uint8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "lock_event_id",     &p_obj)) data.lock_event_id      = (uint64_t)json_object_get_int64(p_obj);
   if (json_object_object_get_ex(p_root, "light_event_id",    &p_obj)) data.light_event_id     = (uint64_t)json_object_get_int64(p_obj);

   data.age_pir = json_get_age(p_root, "age_pir");
   data.age_lgt = json_get_age(p_root, "age_lgt");
   data.age_lck = json_get_age(p_root, "age_lck");

   if (json_object_object_get_ex(p_root, "reeds",     &p_obj)) parse_reed_slots(p_obj,     &data);
   if (json_object_object_get_ex(p_root, "pirs",      &p_obj)) parse_pir_slots(p_obj,      &data);
   if (json_object_object_get_ex(p_root, "temp_count",&p_obj)) data.temp_count = (uint8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_root, "temps",     &p_obj)) parse_temp_slots(p_obj,     &data);
   if (json_object_object_get_ex(p_root, "doorbells", &p_obj)) parse_doorbell_slots(p_obj, &data);
   if (json_object_object_get_ex(p_root, "cams",      &p_obj)) parse_cam_slots(p_obj,      &data);
   if (json_object_object_get_ex(p_root, "rooms",     &p_obj)) parse_rooms(p_obj,          &data);

   json_object_put(p_root);

   pipe_ensure_connected();
   pipe_write(&data, frame_seq);

   /* ---- Condensed summary ---- */
   log_msg("[PARSE] seq=%u tmp=%.1f mot=%u lgt=%d lck=%d occ=%d mtr=%d db=%d "
           "ages pir=%d lgt=%d lck=%d batts pir=%d lck=%d mtr=%d",
           frame_seq,
           data.avg_temp, data.motion_count,
           data.light_state, data.lock_state,
           data.pir_occupied, data.motor_online,
           data.doorbell_pressed,
           data.age_pir, data.age_lgt, data.age_lck,
           data.batt_pir, data.batt_lck, data.batt_motor);

   /* ---- Per-slot PIR (one line, active slots only) ---- */
   for (i = 0; i < MAX_PIRS; i++)
   {
      if (data.pir_slots[i].active)
      {
         log_msg("[PARSE] PIR slot=%d cnt=%u batt=%d age=%d occ=%d eid=%llu",
                 i + 1,
                 data.pir_slots[i].count,
                 data.pir_slots[i].batt,
                 data.pir_slots[i].age,
                 data.pir_slots[i].occupied,
                 (unsigned long long)data.pir_slots[i].event_id);
      }
   }

   /* ---- Per-slot Reed (one line, active slots only) ---- */
   for (i = 0; i < MAX_REEDS; i++)
   {
      if (data.reed_slots[i].active)
      {
         log_msg("[PARSE] Reed slot=%d (%s) st=%s batt=%d age=%d gen=%u eid=%llu",
                 i + 1,
                 data.reed_slots[i].name,
                 (1 == data.reed_slots[i].state) ? "open"   :
                 (0 == data.reed_slots[i].state) ? "closed" : "?",
                 data.reed_slots[i].batt,
                 data.reed_slots[i].age,
                 data.reed_slots[i].gen,
                 (unsigned long long)data.reed_slots[i].event_id);
      }
   }

   /* ---- Per-slot Temp (one line, active slots only) ---- */
   for (i = 0; i < MAX_TEMPS; i++)
   {
      if (data.temp_slots[i].active)
      {
         log_msg("[PARSE] Temp slot=%d %d.%dC batt=%d age=%d eid=%llu",
                 i + 1,
                 data.temp_slots[i].temp_decidegc / 10,
                 data.temp_slots[i].temp_decidegc % 10,
                 data.temp_slots[i].batt,
                 data.temp_slots[i].age,
                 (unsigned long long)data.temp_slots[i].event_id);
      }
   }

   /* ---- Lock / Light event IDs (only when non-zero) ---- */
   if (data.lock_event_id != 0)
      log_msg("[PARSE] Lock st=%d batt=%d eid=%llu",
              data.lock_state, data.batt_lck,
              (unsigned long long)data.lock_event_id);

   if (data.light_event_id != 0)
      log_msg("[PARSE] Light st=%d eid=%llu",
              data.light_state,
              (unsigned long long)data.light_event_id);

   /* ---- Doorbells — one line showing online set only ---- */
   {
      char db_buf[64] = {0};
      int  pos        = 0;
      for (i = 0; i < MAX_DOORBELL_CAMS; i++)
      {
         if (data.doorbell_slots[i].online)
            pos += snprintf(db_buf + pos, sizeof(db_buf) - pos,
                            " id=%d age=%d", i, data.doorbell_slots[i].age_s);
      }
      if (pos > 0)
         log_msg("[PARSE] Doorbells online:%s", db_buf);
   }

   /* ---- Cams — one line showing online set only ---- */
   {
      char cam_buf[64] = {0};
      int  pos         = 0;
      for (i = 0; i < MAX_CAMS; i++)
      {
         if (data.cam_slots[i].online)
            pos += snprintf(cam_buf + pos, sizeof(cam_buf) - pos,
                            " slot=%d age=%d", i, data.cam_slots[i].age_s);
      }
      if (pos > 0)
         log_msg("[PARSE] Cams online:%s", cam_buf);
   }

   /* ---- Rooms ---- */
   for (i = 0; i < data.room_count; i++)
   {
      log_msg("[PARSE] Room id=%d %s/%s=%s",
              data.rooms[i].sensor_id,
              data.rooms[i].room_name,
              data.rooms[i].location,
              data.rooms[i].state);
   }
}
