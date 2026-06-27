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
 *          Structured event tracing — Phase 4A (2026-06-16):
 *          event_id parsed from per-slot JSON into ReedSlotData,
 *          PirSlotData, TempSlotData. lock_event_id and light_event_id
 *          parsed into SensorData. Slot summary logs condensed to one
 *          line per device type.
 *
 *          Wire protocol split — Phase 4B (2026-06-20):
 *          Hub now emits a two-key envelope per tick:
 *
 *            {
 *              "telemetry": { ... },
 *              "events": [ { "type": "...", "slot": N, "event_id": NNN }, ... ]
 *            }
 *
 *          "telemetry" carries all continuous state (ages, battery,
 *          occupancy, slot arrays). Sent every tick regardless of
 *          whether anything changed. High-volume, low-signal.
 *
 *          "events" carries only slots whose event_id advanced since
 *          the hub's last send. Delta filtering is performed once, at
 *          the hub boundary (send_to_bb() / tcp_manager.c), using
 *          per-domain s_last_sent_*_eid[] caches. The hub guarantees
 *          that every entry in events[] represents a true state
 *          transition — novelty is a protocol contract, not a
 *          BeagleBone-side convention.
 *
 *          Consequences for this file:
 *          - All scalars and slot arrays are read from p_root["telemetry"],
 *            not from p_root directly.
 *          - event_id fields are no longer present in per-slot telemetry
 *            objects (reeds[], pirs[], temps[]). parse_reed_slots(),
 *            parse_pir_slots(), parse_temp_slots() skip event_id silently
 *            (key absent — no change needed in those functions).
 *          - lock_event_id and light_event_id are no longer scalar fields
 *            in telemetry; they arrive exclusively via events[].
 *          - parse_events() is the sole writer of all event_id fields in
 *            SensorData. It routes by "type" string into the correct slot
 *            or scalar field.
 *          - No dedup logic is needed or performed here. A duplicate
 *            event_id arriving in events[] would indicate a hub bug or
 *            TCP reconnect replay and is logged as WARN, not silently
 *            suppressed.
 *
 *          Log split (2026-06-20):
 *          Events and telemetry write to separate log files, joined by
 *          frame_seq. See pipe_writer.c for log routing.
 *          - events.log   — one line per events[] entry. Low-volume,
 *                           high-signal. Audit trail. Rotate slowly.
 *          - telemetry.log — one line per frame. Numeric and boolean
 *                           device state only — no strings, no semantic
 *                           decoding. High-volume; rotate aggressively.
 *          Cross-referencing: find frame_seq of an event_id in events.log,
 *          look up that frame_seq in telemetry.log for contemporaneous
 *          battery/age state. Wall-clock timestamp on both is BeagleBone
 *          time(NULL) at parse time — same call, same instant, no
 *          arithmetic needed.
 *
 *          Log split (2026-06-21):
 *          log_msg() calls in the output block replaced with:
 *          - log_telemetry() — one call per frame, fixed numeric schema
 *          - log_event()     — one call per events[] entry
 *          Rooms, reed names, and all string fields removed from telemetry.
 *          Room and reed string state belongs in events.log if needed.
 *          [PARSE] summary line retained in sensor_server.log for
 *          operational visibility (quick grep of the main log still works).
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
 *
 * \note  event_id is intentionally absent from telemetry slot objects
 *        as of Phase 4B. parse_events() is the sole writer of
 *        reed_slots[].event_id. The json_object_object_get_ex() call
 *        for "event_id" that existed here has been removed.
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
   }
}

/******************************************************************************
 * \brief Parse PIR slots from JSON pirs array into SensorData.
 *
 * \note  event_id is intentionally absent from telemetry slot objects
 *        as of Phase 4B. parse_events() is the sole writer of
 *        pir_slots[].event_id.
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
   }
}

/******************************************************************************
 * \brief Parse "temps" JSON array into p_data->temp_slots[].
 *
 * \note  event_id is intentionally absent from telemetry slot objects
 *        as of Phase 4B. parse_events() is the sole writer of
 *        temp_slots[].event_id.
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
 * \brief Parse "events" array from hub delta gate into SensorData.
 *
 * \details Hub guarantees novelty — every entry here represents a true
 *          state transition. No dedup is performed on this side.
 *
 *          A duplicate event_id arriving here indicates a hub bug or
 *          TCP reconnect replay. Callers should treat a repeat as WARN,
 *          not silent suppression.
 *
 *          Entry formats:
 *            { "type": "PIR",   "slot": N, "event_id": NNN }
 *            { "type": "REED",  "slot": N, "event_id": NNN }
 *            { "type": "TEMP",  "slot": N, "event_id": NNN }
 *            { "type": "LOCK",             "event_id": NNN }
 *            { "type": "LIGHT",            "event_id": NNN }
 *
 *          Unknown "type" values are silently skipped (forward compatible).
 ******************************************************************************/
static void parse_events(struct json_object *p_events_arr,
                         struct SensorData  *p_data)
{
   int                 n        = 0;
   int                 i        = 0;
   int                 slot     = 0;
   uint64_t            event_id = 0;
   const char         *p_type   = NULL;
   struct json_object *p_entry  = NULL;
   struct json_object *p_jval   = NULL;

   n = json_object_array_length(p_events_arr);

   for (i = 0; i < n; i++)
   {
      p_entry = json_object_array_get_idx(p_events_arr, i);
      if (NULL == p_entry) { continue; }

      if (!json_object_object_get_ex(p_entry, "type", &p_jval)) { continue; }
      p_type = json_object_get_string(p_jval);
      if (NULL == p_type) { continue; }

      if (!json_object_object_get_ex(p_entry, "event_id", &p_jval)) { continue; }
      event_id = (uint64_t)json_object_get_int64(p_jval);

      if (0 == strcmp(p_type, "LOCK"))
      {
         p_data->lock_event_id = event_id;
      }
      else if (0 == strcmp(p_type, "LIGHT"))
      {
         p_data->light_event_id = event_id;
      }
      else
      {
         /* Slot-based types: PIR, REED, TEMP */
         if (!json_object_object_get_ex(p_entry, "slot", &p_jval)) { continue; }
         slot = json_object_get_int(p_jval) - 1;

         if (0 == strcmp(p_type, "PIR"))
         {
            if ((slot >= 0) && (slot < MAX_PIRS))
               p_data->pir_slots[slot].event_id = event_id;
         }
         else if (0 == strcmp(p_type, "REED"))
         {
            if ((slot >= 0) && (slot < MAX_REEDS))
               p_data->reed_slots[slot].event_id = event_id;
         }
         else if (0 == strcmp(p_type, "TEMP"))
         {
            if ((slot >= 0) && (slot < MAX_TEMPS))
               p_data->temp_slots[slot].event_id = event_id;
         }
         /* Unknown type: silently skip — forward compatible */
      }
   }
}

/******************************************************************************
 * \brief Helper — return age as int for log formatting.
 *
 * \details AGE_UNKNOWN (65534) renders as -1 in log output so offline
 *          devices are visually consistent with batt=-1 (no-read sentinel).
 *          Raw value is preserved in SensorData; this is display-only.
 ******************************************************************************/
static int fmt_age(uint16_t age)
{
   return (age >= AGE_UNKNOWN) ? -1 : (int)age;
}

/******************************************************************************
 * \brief Emit one telemetry.log line for this frame.
 *
 * \details Fixed field order, every device every frame, numeric/boolean
 *          only. No strings, no semantic decoding. Age sentinel 65534
 *          rendered as -1.
 *
 *          Field order:
 *            seq
 *            lck  age_lck
 *            lgt  age_lgt
 *            pir[0..MAX_PIRS-1]  occ age batt
 *            reed[0..MAX_REEDS-1] state age batt
 *            temp[0..MAX_TEMPS-1] val age batt
 *            cam[0..MAX_CAMS-1]  online age
 *            db[0..MAX_DOORBELL_CAMS-1] online age
 ******************************************************************************/
static void emit_telemetry(const struct SensorData *p_data, uint32_t seq)
{
   /* Build line into a local buffer so log_telemetry() makes one write. */
   char  buf[2048];
   int   pos = 0;
   int   i   = 0;

   pos += snprintf(buf + pos, sizeof(buf) - pos,
                   "seq=%u"
                   " lck=%d age_lck=%d"
                   " lgt=%d age_lgt=%d",
                   seq,
                   p_data->lock_state,  fmt_age(p_data->age_lck),
                   p_data->light_state, fmt_age(p_data->age_lgt));

   for (i = 0; i < MAX_PIRS; i++)
   {
      pos += snprintf(buf + pos, sizeof(buf) - pos,
                      " pir%d_occ=%d pir%d_age=%d pir%d_batt=%d",
                      i + 1, p_data->pir_slots[i].occupied,
                      i + 1, fmt_age(p_data->pir_slots[i].age),
                      i + 1, p_data->pir_slots[i].batt);
   }

   for (i = 0; i < MAX_REEDS; i++)
   {
      /* state=0xFF means slot never populated — emit as -1 */
      int st = (p_data->reed_slots[i].active) ? p_data->reed_slots[i].state : -1;
      pos += snprintf(buf + pos, sizeof(buf) - pos,
                      " reed%d=%d reed%d_age=%d reed%d_batt=%d",
                      i + 1, st,
                      i + 1, fmt_age(p_data->reed_slots[i].age),
                      i + 1, p_data->reed_slots[i].batt);
   }

   for (i = 0; i < MAX_TEMPS; i++)
   {
      pos += snprintf(buf + pos, sizeof(buf) - pos,
                      " temp%d=%d temp%d_age=%d temp%d_batt=%d",
                      i + 1, p_data->temp_slots[i].temp_decidegc,
                      i + 1, fmt_age(p_data->temp_slots[i].age),
                      i + 1, p_data->temp_slots[i].batt);
   }

   log_telemetry("%s", buf);
}

/******************************************************************************
 * \brief Emit one events.log line per events[] entry in this frame.
 *
 * \details Slot-based and scalar events. frame_seq is the join key back
 *          to telemetry.log. One call per non-zero event_id field.
 ******************************************************************************/
static void emit_events(const struct SensorData *p_data, uint32_t seq)
{
   int i = 0;

   for (i = 0; i < MAX_PIRS; i++)
   {
      if (p_data->pir_slots[i].event_id != 0)
         log_event("seq=%u type=PIR slot=%d event_id=%llu",
                   seq, i + 1,
                   (unsigned long long)p_data->pir_slots[i].event_id);
   }

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (p_data->reed_slots[i].event_id != 0)
         log_event("seq=%u type=REED slot=%d event_id=%llu",
                   seq, i + 1,
                   (unsigned long long)p_data->reed_slots[i].event_id);
   }

   for (i = 0; i < MAX_TEMPS; i++)
   {
      if (p_data->temp_slots[i].event_id != 0)
         log_event("seq=%u type=TEMP slot=%d event_id=%llu",
                   seq, i + 1,
                   (unsigned long long)p_data->temp_slots[i].event_id);
   }

   if (p_data->lock_event_id != 0)
      log_event("seq=%u type=LOCK event_id=%llu",
                seq, (unsigned long long)p_data->lock_event_id);

   if (p_data->light_event_id != 0)
      log_event("seq=%u type=LIGHT event_id=%llu",
                seq, (unsigned long long)p_data->light_event_id);
}

/******************************************************************************
 * \brief Parse a JSON body string into SensorData and write to pipe.
 *
 * \details Expects the Phase 4B envelope:
 *            { "telemetry": { ... }, "events": [ ... ] }
 *
 *          Rejects frames missing "telemetry" with a [PARSE] log line
 *          rather than falling back to flat-root parsing. A missing
 *          "telemetry" key means a wrong-protocol-version frame from
 *          a hub that has not yet been updated; silent misrouting is
 *          a worse failure mode than explicit rejection.
 *
 *          "events" may be an empty array — this is normal on telemetry-
 *          only ticks and must be handled gracefully.
 ******************************************************************************/
void process_json(const char *p_json_body)
{
   struct json_object *p_root      = NULL;
   struct json_object *p_telemetry = NULL;
   struct json_object *p_obj       = NULL;
   struct SensorData   data;
   int                 i           = 0;
   uint32_t            frame_seq   = 0;

   frame_seq = ++g_frame_seq;

   p_root = json_tokener_parse(p_json_body);
   if (NULL == p_root)
   {
      log_msg("[PARSE] decode_failed frame_seq=%u", frame_seq);
      return;
   }

   /* Require telemetry envelope — reject old-protocol flat frames */
   if (!json_object_object_get_ex(p_root, "telemetry", &p_telemetry))
   {
      log_msg("[PARSE] missing_telemetry frame_seq=%u -- wrong protocol version?",
              frame_seq);
      json_object_put(p_root);
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
      data.reed_slots[i].age      = AGE_UNKNOWN;
      data.reed_slots[i].batt     = -1;
      data.reed_slots[i].active   = 0;
      data.reed_slots[i].state    = 0xFF;
      data.reed_slots[i].offline  = 0;
      data.reed_slots[i].gen      = 0;
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

   data.lock_event_id  = 0;
   data.light_event_id = 0;

   /* ---- Telemetry scalars ---- */
   if (json_object_object_get_ex(p_telemetry, "avg_temp",           &p_obj)) data.avg_temp          = json_object_get_double(p_obj);
   if (json_object_object_get_ex(p_telemetry, "motion_count",       &p_obj)) data.motion_count       = json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "light_state",        &p_obj)) data.light_state        = json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "lock_state",         &p_obj)) data.lock_state         = json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "batt_pir",           &p_obj)) data.batt_pir           = (int8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "pir_occupied",       &p_obj)) data.pir_occupied       = (int8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "doorbell_pressed",   &p_obj)) data.doorbell_pressed   = (uint8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "doorbell_device_id", &p_obj)) data.doorbell_device_id = (uint8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "batt_lck",           &p_obj)) data.batt_lck           = (int8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "motor_online",       &p_obj)) data.motor_online       = (uint8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "batt_motor",         &p_obj)) data.batt_motor         = json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "pir_count",          &p_obj)) data.pir_count          = (uint8_t)json_object_get_int(p_obj);

   data.age_pir = json_get_age(p_telemetry, "age_pir");
   data.age_lgt = json_get_age(p_telemetry, "age_lgt");
   data.age_lck = json_get_age(p_telemetry, "age_lck");

   /* ---- Telemetry arrays ---- */
   if (json_object_object_get_ex(p_telemetry, "reeds",      &p_obj)) parse_reed_slots(p_obj,     &data);
   if (json_object_object_get_ex(p_telemetry, "pirs",       &p_obj)) parse_pir_slots(p_obj,      &data);
   if (json_object_object_get_ex(p_telemetry, "temp_count", &p_obj)) data.temp_count = (uint8_t)json_object_get_int(p_obj);
   if (json_object_object_get_ex(p_telemetry, "temps",      &p_obj)) parse_temp_slots(p_obj,     &data);
   if (json_object_object_get_ex(p_telemetry, "rooms",      &p_obj)) parse_rooms(p_obj,          &data);

   /* ---- Events[] — hub delta gate, novelty guaranteed by protocol ---- */
   if (json_object_object_get_ex(p_root, "events", &p_obj)) parse_events(p_obj, &data);

   json_object_put(p_root);

   pipe_ensure_connected();
   pipe_write(&data, frame_seq);

   /* ---- Operational summary to sensor_server.log (grep-friendly) ---- */
   log_msg("[PARSE] seq=%u tmp=%.1f mot=%u lgt=%d lck=%d occ=%d mtr=%d db=%d "
           "ages pir=%d lgt=%d lck=%d batts pir=%d lck=%d mtr=%d",
           frame_seq,
           data.avg_temp, data.motion_count,
           data.light_state, data.lock_state,
           data.pir_occupied, data.motor_online,
           data.doorbell_pressed,
           fmt_age(data.age_pir), fmt_age(data.age_lgt), fmt_age(data.age_lck),
           data.batt_pir, data.batt_lck, data.batt_motor);

   /* ---- Routed output ---- */
   emit_telemetry(&data, frame_seq);
   emit_events(&data, frame_seq);
}
