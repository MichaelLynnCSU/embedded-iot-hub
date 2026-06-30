/******************************************************************************
 * \file tcp_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief TCP manager for ESP32 hub node.
 *
 * \details Maintains a non-blocking TCP client connection to BeagleBone.
 *          Drains wroom bus queues and sends consolidated JSON payloads
 *          every TCP_SEND_INTERVAL_MS.
 *
 *          BeagleBone connection (outbound client):
 *          State 0 — disconnected, create socket and initiate connect
 *          State 1 — connect in progress, poll with select()
 *          State 2 — connected, ready to send
 *
 *          Motor server: delegated to motor_server.c / motor_server.h.
 *
 *          Backpressure handling (BeagleBone only):
 *          - EAGAIN/EWOULDBLOCK increments block counter
 *          - 5 consecutive blocks triggers force reconnect
 *
 * \note    Phantom widget fix (2026-04-21): (unchanged)
 * \note    WDT fix (2026-03-21): (unchanged)
 * \note    Motor battery (2026-04-08, updated 2026-04-27): (unchanged)
 * \note    PI controller + motor state machine (2026-05-04): (unchanged)
 * \note    Motor server flip (2026-05-XX): motor paths moved to motor_server.c
 * \note    Per-slot PIR occupancy (2026-05-20): per-slot occupied/offline in
 *          PIR_SLOT_STATE_T; flat g_state.pir_occupied retained for JSON
 *          backward compat (slot 0).
 *          0->1 transition detection remains here in drain_queues().
 * \note    TempSensor BLE (2026-06-02): BLE temp sensor slot table wired in.
 *          Mirrors reed/PIR pattern. JSON "temp" field is whole degrees C
 *          matching "avg_temp" from UART/blue pill — divide by 10 at
 *          serialization point only, int16_t kept throughout.
 * \note    Remoted mailbox point-to-point queues (2026-06-16): Centralized
 *          ownership shifted exclusively to event_aggregator.c.
 * \note    Structured event tracing — Phase 4A (2026-06-16):
 *          drain_queues() captures event_id per slot from BLE mailboxes
 *          into g_state before sync_system_state(). Per-slot event_id
 *          included in JSON and logged at TCP send time.
 ******************************************************************************/

#include "config.h"
#include "network_config.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include "ble_manager.h"
#include "ble_internal.h"
#include "ble_pir.h"
#include "ble_reed.h"
#include "ble_temp.h"
#include "uart_manager.h"
#include "tcp_manager.h"
#include "device_gateway.h"
#include "wroom_bus.h"
#include "trinity_log.h"
#include "pi_controller.h"
#include "motor_sm.h"
#include "pir_window.h"
#include "motor_server.h"

#define BB_CONNECT_TIMEOUT_MS   2000
#define BLOCK_COUNT_MAX         5
#define SOCK_POLL_DELAY_MS      100
#define SOCK_RETRY_DELAY_MS     1000
#define REED_NAME_BUF_SIZE      32
#define SOCK_INVALID            -1
#define TCP_STATE_DISCONNECTED  0
#define TCP_STATE_CONNECTING    1
#define TCP_STATE_CONNECTED     2

static const char *TAG = "TCP_MGR";

/* ---- Per-domain last-sent event_id caches (delta gate) ---- */
static uint64_t s_last_sent_pir_eid[MAX_PIRS];
static uint64_t s_last_sent_reed_eid[MAX_REEDS];
static uint64_t s_last_sent_temp_eid[MAX_TEMPS];
static uint64_t s_last_sent_lock_eid;
static uint64_t s_last_sent_light_eid;

/*----------------------------------------------------------------------------*/

typedef struct
{
   int      batt;
   uint16_t age;
   bool     active;
   uint8_t  state;
   uint8_t  offline;
   uint16_t gen;
   uint64_t event_id;   /*!< last wroom event_id for this slot */
} REED_SLOT_STATE_T;

typedef struct
{
   int      batt;
   uint32_t motion_count;
   uint16_t age;
   bool     active;
   int      occupied;
   uint8_t  offline;
   uint64_t event_id;   /*!< last wroom event_id for this slot */
} PIR_SLOT_STATE_T;

typedef struct
{
   int16_t  temp_decidegc;
   int      batt;
   uint16_t age;
   bool     active;
   uint8_t  offline;
   uint16_t gen;
   uint64_t event_id;   /*!< last wroom event_id for this slot */
} TEMP_SLOT_STATE_T;

typedef struct
{
   int               avg_temp;
   uint32_t          motion_count;
   int               pir_batt;
   int               pir_occupied;
   uint8_t           doorbell_pressed;
   uint8_t           doorbell_device_id;
   uint8_t           light_state;
   uint8_t           lock_state;
   int               lock_batt;
   uint8_t           motor_online;
   int               motor_batt;
   REED_SLOT_STATE_T reed_slots[MAX_REEDS];
   PIR_SLOT_STATE_T  pir_slots[MAX_PIRS];
   TEMP_SLOT_STATE_T temp_slots[MAX_TEMPS];
   uint16_t          age_pir;
   uint16_t          age_lgt;
   uint16_t          age_lck;
   int               reed_count;
   int               pir_count;
   int               temp_count;
   uint64_t          lock_event_id;   /*!< last event_id for BLE lock  */
   uint64_t          light_event_id;  /*!< last event_id for BLE light */
} TCP_STATE_T;

static TCP_STATE_T g_state =
{
   .avg_temp           = DEFAULT_AVG_TEMP,
   .pir_batt           = -1,
   .motion_count       = 0,
   .pir_occupied       = 0,
   .doorbell_pressed   = 0,
   .doorbell_device_id = 0,
   .lock_batt          = -1,
   .motor_batt         = -1,
   .age_pir            = 0xFFFF,
   .age_lgt            = 0xFFFF,
   .age_lck            = 0xFFFF,
   .reed_count         = 0,
   .pir_count          = 0,
   .temp_count         = 0,
};

/*----------------------------------------------------------------------------*/

static void drain_queues(BUS_SUBSCRIBER_T *p_sub)
{
   PIR_PAYLOAD_T      pir  = {0};
   REED_PAYLOAD_T     reed = {0};
   LOCK_PAYLOAD_T     lock = {0};
   LIGHT_PAYLOAD_T    lgt  = {0};
   BLE_TEMP_PAYLOAD_T tmp  = {0};

   if (pdTRUE == xQueueReceive(p_sub->mb_pir, &pir, 0))
   {
      int s = (int)pir.slot - 1;
      if ((s >= 0) && (s < MAX_PIRS))
      {
         g_state.pir_slots[s].event_id = pir.event_id;
      }
   }

   if (pdTRUE == xQueueReceive(p_sub->mb_reed, &reed, 0))
   {
      int s = (int)reed.id - 1;
      if ((s >= 0) && (s < MAX_REEDS))
      {
         g_state.reed_slots[s].event_id = reed.event_id;
      }
   }

   if (pdTRUE == xQueueReceive(p_sub->mb_lock, &lock, 0))
   {
      g_state.lock_event_id = lock.event_id;
   }

   if (pdTRUE == xQueueReceive(p_sub->mb_light, &lgt, 0))
   {
      g_state.light_event_id = lgt.event_id;
   }

   if (pdTRUE == xQueueReceive(p_sub->mb_ble_temp, &tmp, 0))
   {
      int s = (int)tmp.id - 1;
      if ((s >= 0) && (s < MAX_TEMPS))
      {
         g_state.temp_slots[s].event_id = tmp.event_id;
      }
   }
}

/*----------------------------------------------------------------------------*/

static void sync_system_state(void)
{
   int i = 0;

   g_state.age_pir = ble_get_device_age_s(BLE_DEV_PIR);
   g_state.age_lgt = ble_get_device_age_s(DEV_IDX_LIGHT);
   g_state.age_lck = ble_get_device_age_s(DEV_IDX_LOCK);

   int      count      = ble_get_reed_count();
   uint16_t age        = 0xFFFF;
   uint8_t  slot_state = 0xFF;
   uint16_t gen        = 0;

   for (i = 0; (i < count) && (i < MAX_REEDS); i++)
   {
      age = 0xFFFF; slot_state = 0xFF; gen = 0;
      (void)ble_get_reed_slot_info(i, NULL, NULL, &age, &slot_state, &gen);
      g_state.reed_slots[i].age     = age;
      g_state.reed_slots[i].active  = true;
      g_state.reed_slots[i].state   = slot_state;
      g_state.reed_slots[i].offline = (age > REED_OFFLINE_S) ? 1 : 0;
      g_state.reed_slots[i].gen     = gen;
   }

   g_state.pir_count    = ble_pir_get_count();
   g_state.motion_count = 0;

   for (i = 0; (i < g_state.pir_count) && (i < MAX_PIRS); i++)
   {
      age = 0xFFFF;
      int batt_pir = 0;
      (void)ble_pir_get_slot_info(i, NULL, &batt_pir, &age);
      g_state.pir_slots[i].batt     = batt_pir;
      g_state.pir_slots[i].age      = age;
      g_state.pir_slots[i].active   = true;
      g_state.pir_slots[i].occupied = pir_window_get_occupied(i);
      g_state.pir_slots[i].offline  = (age > PIR_OFFLINE_S) ? 1 : 0;

      if (g_state.pir_slots[i].active)
      {
         g_state.motion_count += g_state.pir_slots[i].motion_count;
      }
   }

   g_state.pir_occupied = (g_state.pir_count > 0) ?
                           g_state.pir_slots[0].occupied : 0;

   g_state.temp_count = ble_get_temp_count();
   for (i = 0; (i < g_state.temp_count) && (i < MAX_TEMPS); i++)
   {
      int16_t  t_decidegc = 0;
      int      t_batt     = -1;
      uint16_t t_age      = 0xFFFF;
      uint16_t t_gen      = 0;
      (void)ble_get_temp_slot_info(i, NULL, &t_decidegc, &t_batt, &t_age, &t_gen);
      g_state.temp_slots[i].temp_decidegc = t_decidegc;
      g_state.temp_slots[i].batt          = t_batt;
      g_state.temp_slots[i].age           = t_age;
      g_state.temp_slots[i].active        = true;
      g_state.temp_slots[i].offline       = (t_age > (TEMP_OFFLINE_MS / 1000)) ? 1 : 0;
      g_state.temp_slots[i].gen           = t_gen;
   }

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (g_state.reed_slots[i].active)
      {
         ESP_LOGI(TAG, "[REED] slot=%d state=%d batt=%d age=%d offline=%d gen=%d",
                  i + 1,
                  g_state.reed_slots[i].state,
                  g_state.reed_slots[i].batt,
                  g_state.reed_slots[i].age,
                  g_state.reed_slots[i].offline,
                  g_state.reed_slots[i].gen);
      }
   }

   for (i = 0; i < g_state.pir_count; i++)
   {
      ESP_LOGI(TAG, "[PIR] slot=%d count=%u batt=%d age=%d offline=%d occ=%d",
               i + 1,
               (unsigned)g_state.pir_slots[i].motion_count,
               g_state.pir_slots[i].batt,
               g_state.pir_slots[i].age,
               g_state.pir_slots[i].offline,
               g_state.pir_slots[i].occupied);
   }

   for (i = 0; i < g_state.temp_count; i++)
   {
      ESP_LOGI(TAG, "[TEMP] slot=%d temp=%d.%d°C batt=%d age=%d offline=%d gen=%d",
               i + 1,
               (int)(g_state.temp_slots[i].temp_decidegc / 10),
               (int)(g_state.temp_slots[i].temp_decidegc < 0 ?
                     -(g_state.temp_slots[i].temp_decidegc % 10) :
                       g_state.temp_slots[i].temp_decidegc % 10),
               g_state.temp_slots[i].batt,
               g_state.temp_slots[i].age,
               g_state.temp_slots[i].offline,
               g_state.temp_slots[i].gen);
   }
}

/*----------------------------------------------------------------------------*/

static void handle_bb_send_error(int *p_sock, int *p_block_count, int *p_state)
{
   if ((EAGAIN == errno) || (EWOULDBLOCK == errno))
   {
      (*p_block_count)++;
      ESP_LOGW(TAG, "[BEAGLEBONE] Send would block (%d)", *p_block_count);
      if (*p_block_count >= BLOCK_COUNT_MAX)
      {
         trinity_log_event("EVENT: TCP_BB_FORCE_RECONNECT\n");
         close(*p_sock);
         *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED; *p_block_count = 0;
      }
   }
   else
   {
      trinity_log_event("EVENT: TCP_BB_DISCONNECTED\n");
      close(*p_sock);
      *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED; *p_block_count = 0;
   }
}

/*----------------------------------------------------------------------------*/

static void send_to_bb(int *p_bb_sock, int *p_bb_block_count, int *p_bb_state)
{
   cJSON   *p_root      = NULL;
   cJSON   *p_telemetry = NULL;
   cJSON   *p_events    = NULL;
   cJSON   *p_reeds     = NULL;
   cJSON   *p_rooms     = NULL;
   cJSON   *p_entry     = NULL;
   cJSON   *p_pirs      = NULL;
   cJSON   *p_temps     = NULL;
   char    *p_msg       = NULL;
   char     name[REED_NAME_BUF_SIZE] = {0};
   uint8_t  door_state  = 0xFF;
   uint16_t gen         = 0;
   int      i           = 0;
   int      sent        = 0;

   if ((TCP_STATE_CONNECTED != *p_bb_state) || (SOCK_INVALID == *p_bb_sock)) { return; }

   p_root = cJSON_CreateObject();
   if (NULL == p_root) { ESP_LOGE(TAG, "cJSON root alloc failed (BB)"); return; }

   /* ------------------------------------------------------------------ */
   /* telemetry{} — continuous state, sent every tick regardless of delta */
   /* ------------------------------------------------------------------ */
   p_telemetry = cJSON_CreateObject();
   if (NULL == p_telemetry) { cJSON_Delete(p_root); return; }

   /* Scalar telemetry fields */
   (void)cJSON_AddNumberToObject(p_telemetry, "avg_temp",           g_state.avg_temp);
   (void)cJSON_AddNumberToObject(p_telemetry, "motion_count",       g_state.motion_count);
   (void)cJSON_AddNumberToObject(p_telemetry, "light_state",        g_state.light_state);
   (void)cJSON_AddNumberToObject(p_telemetry, "lock_state",         g_state.lock_state);
   (void)cJSON_AddNumberToObject(p_telemetry, "age_pir",            g_state.age_pir);
   (void)cJSON_AddNumberToObject(p_telemetry, "age_lgt",            g_state.age_lgt);
   (void)cJSON_AddNumberToObject(p_telemetry, "age_lck",            g_state.age_lck);
   (void)cJSON_AddNumberToObject(p_telemetry, "batt_pir",           g_state.pir_batt);
   (void)cJSON_AddNumberToObject(p_telemetry, "pir_occupied",       g_state.pir_occupied);
   (void)cJSON_AddNumberToObject(p_telemetry, "doorbell_pressed",   g_state.doorbell_pressed);
   (void)cJSON_AddNumberToObject(p_telemetry, "doorbell_device_id", g_state.doorbell_device_id);
   (void)cJSON_AddNumberToObject(p_telemetry, "batt_lck",           g_state.lock_batt);
   (void)cJSON_AddNumberToObject(p_telemetry, "batt_motor",         g_state.motor_batt);
   (void)cJSON_AddNumberToObject(p_telemetry, "motor_online",       g_state.motor_online);
   (void)cJSON_AddNumberToObject(p_telemetry, "pir_count",          g_state.pir_count);
   (void)cJSON_AddNumberToObject(p_telemetry, "temp_count",         g_state.temp_count);

   /* Reeds */
   g_state.reed_count = ble_get_reed_count();
   p_reeds = cJSON_CreateArray();
   if (NULL != p_reeds)
   {
      for (i = 0; (i < g_state.reed_count) && (i < MAX_REEDS); i++)
      {
         (void)memset(name, 0, sizeof(name));
         door_state = 0xFF; gen = 0;
         (void)ble_get_reed_slot_info(i, name, NULL, NULL, &door_state, &gen);
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddNumberToObject(p_entry, "id",      i + 1);
            (void)cJSON_AddNumberToObject(p_entry, "batt",    g_state.reed_slots[i].batt);
            (void)cJSON_AddNumberToObject(p_entry, "age",     g_state.reed_slots[i].age);
            (void)cJSON_AddNumberToObject(p_entry, "state",   door_state);
            (void)cJSON_AddNumberToObject(p_entry, "offline", g_state.reed_slots[i].offline);
            (void)cJSON_AddNumberToObject(p_entry, "gen",     gen);
            (void)cJSON_AddStringToObject(p_entry, "name",    name);
            (void)cJSON_AddItemToArray(p_reeds, p_entry);
         }
      }
      (void)cJSON_AddItemToObject(p_telemetry, "reeds", p_reeds);
   }

   /* Rooms */
   p_rooms = cJSON_CreateArray();
   if (NULL != p_rooms)
   {
      for (i = 0; i < ROOM_COUNT; i++)
      {
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddNumberToObject(p_entry, "sensor_id", rooms[i].sensor_id);
            (void)cJSON_AddStringToObject(p_entry, "room",      rooms[i].room);
            (void)cJSON_AddStringToObject(p_entry, "state",     rooms[i].state);
            (void)cJSON_AddStringToObject(p_entry, "location",  rooms[i].location);
            (void)cJSON_AddItemToArray(p_rooms, p_entry);
         }
      }
      (void)cJSON_AddItemToObject(p_telemetry, "rooms", p_rooms);
   }

   /* PIRs */
   p_pirs = cJSON_CreateArray();
   if (NULL != p_pirs)
   {
      for (i = 0; (i < g_state.pir_count) && (i < MAX_PIRS); i++)
      {
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddNumberToObject(p_entry, "id",       i + 1);
            (void)cJSON_AddNumberToObject(p_entry, "count",    g_state.pir_slots[i].motion_count);
            (void)cJSON_AddNumberToObject(p_entry, "batt",     g_state.pir_slots[i].batt);
            (void)cJSON_AddNumberToObject(p_entry, "age",      g_state.pir_slots[i].age);
            (void)cJSON_AddNumberToObject(p_entry, "occupied", g_state.pir_slots[i].occupied);
            (void)cJSON_AddNumberToObject(p_entry, "offline",  g_state.pir_slots[i].offline);
            (void)cJSON_AddItemToArray(p_pirs, p_entry);
         }
      }
      (void)cJSON_AddItemToObject(p_telemetry, "pirs", p_pirs);
   }

   /* Temps */
   p_temps = cJSON_CreateArray();
   if (NULL != p_temps)
   {
      for (i = 0; (i < g_state.temp_count) && (i < MAX_TEMPS); i++)
      {
         char     t_name[ADV_NAME_BUF_SIZE] = {0};
         uint16_t t_gen                     = 0;
         (void)ble_get_temp_slot_info(i, t_name, NULL, NULL, NULL, &t_gen);
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddNumberToObject(p_entry, "id",      i + 1);
            (void)cJSON_AddNumberToObject(p_entry, "temp",    g_state.temp_slots[i].temp_decidegc / 10);
            (void)cJSON_AddNumberToObject(p_entry, "batt",    g_state.temp_slots[i].batt);
            (void)cJSON_AddNumberToObject(p_entry, "age",     g_state.temp_slots[i].age);
            (void)cJSON_AddNumberToObject(p_entry, "offline", g_state.temp_slots[i].offline);
            (void)cJSON_AddNumberToObject(p_entry, "gen",     t_gen);
            (void)cJSON_AddStringToObject(p_entry, "name",    t_name);
            (void)cJSON_AddItemToArray(p_temps, p_entry);
         }
      }
      (void)cJSON_AddItemToObject(p_telemetry, "temps", p_temps);
   }

   (void)cJSON_AddItemToObject(p_root, "telemetry", p_telemetry);

   /* ------------------------------------------------------------------ */
   /* events[] — delta gate: emit only slots whose event_id advanced      */
   /* ------------------------------------------------------------------ */
   p_events = cJSON_CreateArray();
   if (NULL == p_events) { cJSON_Delete(p_root); return; }

   for (i = 0; (i < g_state.pir_count) && (i < MAX_PIRS); i++)
   {
      if (g_state.pir_slots[i].event_id != s_last_sent_pir_eid[i])
      {
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddStringToObject(p_entry, "type",     "PIR");
            (void)cJSON_AddNumberToObject(p_entry, "slot",     i + 1);
            (void)cJSON_AddNumberToObject(p_entry, "event_id", (double)g_state.pir_slots[i].event_id);
            (void)cJSON_AddItemToArray(p_events, p_entry);
            ESP_LOGI(TAG, "[EVENT] PIR slot=%d event_id=%llu",
                     i + 1, (unsigned long long)g_state.pir_slots[i].event_id);
         }
         s_last_sent_pir_eid[i] = g_state.pir_slots[i].event_id;
      }
   }

   for (i = 0; (i < g_state.reed_count) && (i < MAX_REEDS); i++)
   {
      if (g_state.reed_slots[i].event_id != s_last_sent_reed_eid[i])
      {
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddStringToObject(p_entry, "type",     "REED");
            (void)cJSON_AddNumberToObject(p_entry, "slot",     i + 1);
            (void)cJSON_AddNumberToObject(p_entry, "event_id", (double)g_state.reed_slots[i].event_id);
            (void)cJSON_AddItemToArray(p_events, p_entry);
            ESP_LOGI(TAG, "[EVENT] REED slot=%d event_id=%llu state=%d",
                     i + 1, (unsigned long long)g_state.reed_slots[i].event_id,
                     g_state.reed_slots[i].state);
         }
         s_last_sent_reed_eid[i] = g_state.reed_slots[i].event_id;
      }
   }

   for (i = 0; (i < g_state.temp_count) && (i < MAX_TEMPS); i++)
   {
      if (g_state.temp_slots[i].event_id != s_last_sent_temp_eid[i])
      {
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddStringToObject(p_entry, "type",     "TEMP");
            (void)cJSON_AddNumberToObject(p_entry, "slot",     i + 1);
            (void)cJSON_AddNumberToObject(p_entry, "event_id", (double)g_state.temp_slots[i].event_id);
            (void)cJSON_AddItemToArray(p_events, p_entry);
            ESP_LOGI(TAG, "[EVENT] TEMP slot=%d event_id=%llu temp_dc=%d",
                     i + 1, (unsigned long long)g_state.temp_slots[i].event_id,
                     g_state.temp_slots[i].temp_decidegc);
         }
         s_last_sent_temp_eid[i] = g_state.temp_slots[i].event_id;
      }
   }

   if (g_state.lock_event_id != s_last_sent_lock_eid)
   {
      p_entry = cJSON_CreateObject();
      if (NULL != p_entry)
      {
         (void)cJSON_AddStringToObject(p_entry, "type",     "LOCK");
         (void)cJSON_AddNumberToObject(p_entry, "event_id", (double)g_state.lock_event_id);
         (void)cJSON_AddItemToArray(p_events, p_entry);
         ESP_LOGI(TAG, "[EVENT] LOCK event_id=%llu state=%d",
                  (unsigned long long)g_state.lock_event_id, g_state.lock_state);
      }
      s_last_sent_lock_eid = g_state.lock_event_id;
   }

   if (g_state.light_event_id != s_last_sent_light_eid)
   {
      p_entry = cJSON_CreateObject();
      if (NULL != p_entry)
      {
         (void)cJSON_AddStringToObject(p_entry, "type",     "LIGHT");
         (void)cJSON_AddNumberToObject(p_entry, "event_id", (double)g_state.light_event_id);
         (void)cJSON_AddItemToArray(p_events, p_entry);
         ESP_LOGI(TAG, "[EVENT] LIGHT event_id=%llu state=%d",
                  (unsigned long long)g_state.light_event_id, g_state.light_state);
      }
      s_last_sent_light_eid = g_state.light_event_id;
   }

   (void)cJSON_AddItemToObject(p_root, "events", p_events);

   /* ------------------------------------------------------------------ */
   /* Serialize and send                                                   */
   /* ------------------------------------------------------------------ */
   p_msg = cJSON_PrintUnformatted(p_root);
   cJSON_Delete(p_root);
   if (NULL == p_msg) { ESP_LOGE(TAG, "cJSON serialize failed (BB)"); return; }

   sent = send(*p_bb_sock, p_msg, strlen(p_msg), 0);
   if (0 > sent)
   {
      handle_bb_send_error(p_bb_sock, p_bb_block_count, p_bb_state);
   }
   else
   {
      *p_bb_block_count = 0;
      ESP_LOGI(TAG, "[TCP] publish tmp=%d pir=%u occ=%d lgt=%d lck=%d "
                    "reeds=%d temps=%d mtr=%d batt_mtr=%d batt_pir=%d batt_lck=%d"
                    " doorbell=%d db_id=%d",
               g_state.avg_temp, (unsigned)g_state.motion_count,
               g_state.pir_occupied,
               g_state.light_state, g_state.lock_state,
               g_state.reed_count, g_state.temp_count,
               (int)g_state.motor_online, g_state.motor_batt,
               g_state.pir_batt, g_state.lock_batt,
               g_state.doorbell_pressed, g_state.doorbell_device_id);
   }

   /* Clear doorbell pulse — one-shot, reset after each send */
   g_state.doorbell_pressed = 0;
   cJSON_free(p_msg);
}

/*----------------------------------------------------------------------------*/

static void run_bb_state_machine(int *p_sock, int *p_state, int *p_block_count,
                                  uint32_t *p_connect_start,
                                  struct sockaddr_in *p_addr, uint32_t now)
{
   int flags = 0, ret = 0, err = 0;
   socklen_t el = sizeof(err);
   fd_set writefds, errorfds;
   struct timeval tv;

   switch (*p_state)
   {
      case TCP_STATE_DISCONNECTED:
      {
         if (SOCK_INVALID != *p_sock) { close(*p_sock); *p_sock = SOCK_INVALID; }
         *p_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
         if (0 > *p_sock) { vTaskDelay(pdMS_TO_TICKS(SOCK_RETRY_DELAY_MS)); break; }
         flags = fcntl(*p_sock, F_GETFL, 0);
         ESP_LOGI(TAG, "[TCP] Connecting to %s:%d", BEAGLEBONE_IP, BEAGLEBONE_PORT);
         (void)fcntl(*p_sock, F_SETFL, flags | O_NONBLOCK);
         ret = connect(*p_sock, (struct sockaddr *)p_addr, sizeof(*p_addr));
         if (0 == ret)
         {
            trinity_log_event("EVENT: TCP_BB_CONNECTED\n");
            *p_state = TCP_STATE_CONNECTED; *p_block_count = 0;
            ESP_LOGI(TAG, "[TCP] Connected (immediate)");
         }
         else if (EINPROGRESS == errno)
         {
            ESP_LOGI(TAG, "[TCP] Connect in progress...");
            *p_connect_start = now; *p_state = TCP_STATE_CONNECTING;
         }
         else
         {
            ESP_LOGW(TAG, "[TCP] Connect failed (errno=%d), retrying", errno);
            close(*p_sock); *p_sock = SOCK_INVALID;
            vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS));
         }
         break;
      }
      case TCP_STATE_CONNECTING:
      {
         if ((now - *p_connect_start) > BB_CONNECT_TIMEOUT_MS)
         {
            ESP_LOGW(TAG, "[TCP] Connect timed out after %d ms", BB_CONNECT_TIMEOUT_MS);
            close(*p_sock); *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED;
            vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS)); break;
         }
         FD_ZERO(&writefds); FD_ZERO(&errorfds);
         FD_SET(*p_sock, &writefds); FD_SET(*p_sock, &errorfds);
         tv.tv_sec = 0; tv.tv_usec = 10000;
         if (0 < select(*p_sock + 1, NULL, &writefds, &errorfds, &tv))
         {
            (void)getsockopt(*p_sock, SOL_SOCKET, SO_ERROR, &err, &el);
            if (FD_ISSET(*p_sock, &errorfds) || (0 != err))
            {
               ESP_LOGW(TAG, "[TCP] Connect error (SO_ERROR=%d)", err);
               close(*p_sock); *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED;
               vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS));
            }
            else if (FD_ISSET(*p_sock, &writefds))
            {
               trinity_log_event("EVENT: TCP_BB_CONNECTED\n");
               ESP_LOGI(TAG, "[TCP] Connected");
               *p_state = TCP_STATE_CONNECTED; *p_block_count = 0;
            }
         }
         break;
      }
      case TCP_STATE_CONNECTED: break;
      default: *p_state = TCP_STATE_DISCONNECTED; break;
   }
}

/*----------------------------------------------------------------------------*/

void tcp_manager_init(void)
{
   ESP_LOGI(TAG, "TCP manager initialized");
}

void tcp_manager_task(EventGroupHandle_t p_system_eg,
                      EventGroupHandle_t p_wifi_eg,
                      BUS_SUBSCRIBER_T   sub)
{
   int         motor_listen     = SOCK_INVALID;
   int         motor_client     = SOCK_INVALID;
   int         bb_sock          = SOCK_INVALID;
   int         bb_state         = TCP_STATE_DISCONNECTED;
   int         bb_block_count   = 0;
   uint32_t    bb_connect_start = 0;
   uint32_t    now              = 0;
   float       pi_out           = 0.0f;
   float       eff_pwm          = 0.0f;
   bool        do_connect       = false;
   bool        do_disconnect    = false;

   (void)p_system_eg;
   (void)do_connect;

   struct sockaddr_in bb_addr =
   {
      .sin_family      = AF_INET,
      .sin_port        = htons(BEAGLEBONE_PORT),
      .sin_addr.s_addr = inet_addr(BEAGLEBONE_IP),
   };

   (void)xEventGroupWaitBits(p_wifi_eg, WIFI_CONNECTED_BIT,
                             pdFALSE, pdTRUE, portMAX_DELAY);
   ESP_LOGI(TAG, "TCP task running");

   motor_listen = open_motor_listen_socket();
   if (SOCK_INVALID == motor_listen)
   {
      ESP_LOGE(TAG, "[MOTOR_SRV] Failed to open listen socket -- motor offline");
      g_state.motor_online = 0;
   }

   while (1)
   {
      trinity_wdt_kick();
      now = xTaskGetTickCount() * portTICK_PERIOD_MS;

      pi_out  = run_pi_controller(now);
      eff_pwm = run_motor_sm(pi_out, now, &do_connect, &do_disconnect);

      if (SOCK_INVALID != motor_listen)
      {
         motor_client = accept_motor_connection(motor_listen);
         if (SOCK_INVALID != motor_client)
         {
            send_pwm_to_c3(motor_client, eff_pwm,
                           &g_state.motor_online, &g_state.motor_batt);
            motor_client = SOCK_INVALID;
         }
      }
      else
      {
         motor_listen = open_motor_listen_socket();
         if (SOCK_INVALID == motor_listen)
         {
            g_state.motor_online = 0;
         }
      }

      run_bb_state_machine(&bb_sock, &bb_state, &bb_block_count,
                           &bb_connect_start, &bb_addr, now);

      if (TCP_STATE_CONNECTED != bb_state)
      {
         vTaskDelay(pdMS_TO_TICKS(SOCK_POLL_DELAY_MS));
         continue;
      }

      (void)xEventGroupWaitBits(sub.events, sub.mask, pdTRUE, pdFALSE,
                                pdMS_TO_TICKS(TCP_SEND_INTERVAL_MS));

      drain_queues(&sub);
      sync_system_state();
      send_to_bb(&bb_sock, &bb_block_count, &bb_state);
   }
}
