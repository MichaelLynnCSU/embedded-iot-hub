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
 *          Dependency direction:
 *            json_parser.c -> pipe_writer.c -> named pipe -> controller
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

/******************************************************************************
 * \brief Extract age value from JSON object with range clamping.
 *
 * \param p_root - JSON root object.
 * \param p_key  - Key string to look up.
 *
 * \return uint16_t - Age in seconds, AGE_UNKNOWN if missing or negative,
 *                    clamped to AGE_MAX if too large.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static uint16_t json_get_age(struct json_object *p_root, const char *p_key)
{
   struct json_object *p_obj = NULL; /**< extracted JSON object */
   int                 v     = 0;    /**< integer value */

   if (!json_object_object_get_ex(p_root, p_key, &p_obj))
   {
      return AGE_UNKNOWN;
   }

   v = json_object_get_int(p_obj);

   if (0 > v)
   {
      return AGE_UNKNOWN;
   }

   if (v > (int)AGE_MAX)
   {
      return (uint16_t)AGE_MAX;
   }

   return (uint16_t)v;
}

/******************************************************************************
 * \brief Parse reed slots from JSON reeds array into SensorData.
 *
 * \param p_reeds_arr - JSON array object for "reeds" key.
 * \param p_data      - Pointer to SensorData to populate.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void parse_reed_slots(struct json_object *p_reeds_arr,
                              struct SensorData  *p_data)
{
   int                 n      = 0;    /**< array length */
   int                 i      = 0;    /**< loop index */
   int                 id     = 0;    /**< reed slot id */
   int                 slot   = 0;    /**< slot index */
   int                 age    = 0;    /**< raw age value */
   struct json_object *p_r    = NULL; /**< single reed entry */
   struct json_object *p_jid  = NULL; /**< id field */
   struct json_object *p_jval = NULL; /**< generic value field */

   n = json_object_array_length(p_reeds_arr);

   for (i = 0; i < n; i++)
   {
      p_r = json_object_array_get_idx(p_reeds_arr, i);

      if (!json_object_object_get_ex(p_r, "id", &p_jid))
      {
         continue;
      }

      id   = json_object_get_int(p_jid);
      slot = id - 1;

      if ((0 > slot) || (slot >= MAX_REEDS))
      {
         continue;
      }

      p_data->reed_slots[slot].active = 1;

      if (json_object_object_get_ex(p_r, "batt", &p_jval))
      {
         p_data->reed_slots[slot].batt =
            (int8_t)json_object_get_int(p_jval);
      }

      if (json_object_object_get_ex(p_r, "age", &p_jval))
      {
         age = json_object_get_int(p_jval);
         p_data->reed_slots[slot].age =
            (0 > age) ? AGE_UNKNOWN : (uint16_t)age;
      }

      if (json_object_object_get_ex(p_r, "state", &p_jval))
      {
         p_data->reed_slots[slot].state =
            (uint8_t)json_object_get_int(p_jval);
      }

      if (json_object_object_get_ex(p_r, "offline", &p_jval))
      {
         p_data->reed_slots[slot].offline =
            (uint8_t)json_object_get_int(p_jval);
      }

      if (json_object_object_get_ex(p_r, "gen", &p_jval))
      {
         p_data->reed_slots[slot].gen =
            (uint16_t)json_object_get_int(p_jval);
      }

      if (json_object_object_get_ex(p_r, "name", &p_jval))
      {
         (void)strncpy(p_data->reed_slots[slot].name,
                       json_object_get_string(p_jval),
                       REED_NAME_LEN - 1);
      }

      log_msg("  Reed slot %d (%s): state=%s batt=%d%% age=%d "
              "offline=%d gen=%u",
              slot + 1,
              p_data->reed_slots[slot].name,
              (1 == p_data->reed_slots[slot].state) ? "open"  :
              (0 == p_data->reed_slots[slot].state) ? "closed": "unknown",
              p_data->reed_slots[slot].batt,
              p_data->reed_slots[slot].age,
              p_data->reed_slots[slot].offline,
              p_data->reed_slots[slot].gen);
   }
}

/******************************************************************************
 * \brief Parse room entries from JSON rooms array into SensorData.
 *
 * \param p_rooms_arr - JSON array object for "rooms" key.
 * \param p_data      - Pointer to SensorData to populate.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void parse_rooms(struct json_object *p_rooms_arr,
                         struct SensorData  *p_data)
{
   int                 n      = 0;    /**< array length */
   int                 i      = 0;    /**< loop index */
   struct json_object *p_r    = NULL; /**< single room entry */
   struct json_object *p_jval = NULL; /**< generic value field */

   n = json_object_array_length(p_rooms_arr);

   for (i = 0; (i < n) && (p_data->room_count < MAX_ROOMS); i++)
   {
      p_r = json_object_array_get_idx(p_rooms_arr, i);

      if (!json_object_object_get_ex(p_r, "sensor_id", &p_jval))
      {
         continue;
      }

      p_data->rooms[p_data->room_count].sensor_id =
         json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "room", &p_jval))
      {
         (void)strncpy(p_data->rooms[p_data->room_count].room_name,
                       json_object_get_string(p_jval),
                       ROOM_NAME_LEN - 1);
      }

      if (json_object_object_get_ex(p_r, "state", &p_jval))
      {
         (void)strncpy(p_data->rooms[p_data->room_count].state,
                       json_object_get_string(p_jval),
                       ROOM_STATE_LEN - 1);
      }

      if (json_object_object_get_ex(p_r, "location", &p_jval))
      {
         (void)strncpy(p_data->rooms[p_data->room_count].location,
                       json_object_get_string(p_jval),
                       ROOM_LOC_LEN - 1);
      }

      log_msg("  Room id=%d %s/%s = %s",
              p_data->rooms[p_data->room_count].sensor_id,
              p_data->rooms[p_data->room_count].room_name,
              p_data->rooms[p_data->room_count].location,
              p_data->rooms[p_data->room_count].state);

      p_data->room_count++;
   }
}

/******************************************************************************
 * \brief Parse a JSON body string into SensorData and write to pipe.
 *
 * \param p_json_body - Null-terminated JSON string to parse.
 *                      Must point into a stable buffer (g_process[]) --
 *                      never pass a pointer into the active UART buffer.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void process_json(const char *p_json_body)
{
   struct json_object *p_root = NULL; /**< parsed JSON root */
   struct json_object *p_obj  = NULL; /**< generic field object */
   struct SensorData   data;          /**< output sensor data struct */
   int                 i      = 0;    /**< loop index */

   p_root = json_tokener_parse(p_json_body);
   if (NULL == p_root)
   {
      log_msg("Invalid JSON");
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
   }

   if (json_object_object_get_ex(p_root, "avg_temp", &p_obj))
   {
      data.avg_temp = json_object_get_double(p_obj);
   }

   if (json_object_object_get_ex(p_root, "motion_count", &p_obj))
   {
      data.motion_count = json_object_get_int(p_obj);
   }

   if (json_object_object_get_ex(p_root, "light_state", &p_obj))
   {
      data.light_state = json_object_get_int(p_obj);
   }

   if (json_object_object_get_ex(p_root, "lock_state", &p_obj))
   {
      data.lock_state = json_object_get_int(p_obj);
   }

   data.age_pir = json_get_age(p_root, "age_pir");
   data.age_lgt = json_get_age(p_root, "age_lgt");
   data.age_lck = json_get_age(p_root, "age_lck");

   if (json_object_object_get_ex(p_root, "batt_pir", &p_obj))
   {
      data.batt_pir = (int8_t)json_object_get_int(p_obj);
   }

   if (json_object_object_get_ex(p_root, "pir_occupied", &p_obj))
   {
      data.pir_occupied = (int8_t)json_object_get_int(p_obj);
   }

   if (json_object_object_get_ex(p_root, "batt_lck", &p_obj))
   {
      data.batt_lck = (int8_t)json_object_get_int(p_obj);
   }

   if (json_object_object_get_ex(p_root, "motor_online", &p_obj))
   {
      data.motor_online = (uint8_t)json_object_get_int(p_obj);
      log_msg("motor_online=%d", data.motor_online);
   }

   if (json_object_object_get_ex(p_root, "batt_motor", &p_obj))
   {
      data.batt_motor = json_object_get_int(p_obj);
      log_msg("batt_motor=%d%%", data.batt_motor);
   }

   if (json_object_object_get_ex(p_root, "reeds", &p_obj))
   {
      parse_reed_slots(p_obj, &data);
   }

   log_msg("Parsed avg_temp=%.2f motion=%d light=%d lock=%d",
           data.avg_temp, data.motion_count,
           data.light_state, data.lock_state);
   log_msg("Ages pir=%d lgt=%d lck=%d | Batt pir=%d%% lck=%d%% | Occupied=%d",
           data.age_pir, data.age_lgt, data.age_lck,
           data.batt_pir, data.batt_lck, data.pir_occupied);

   if (json_object_object_get_ex(p_root, "rooms", &p_obj))
   {
      parse_rooms(p_obj, &data);
   }

   json_object_put(p_root);

   pipe_ensure_connected();
   pipe_write(&data);
}
