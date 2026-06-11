/******************************************************************************
 * \file tcp_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief TCP manager for ESP32 hub node.
 *
 * \details Maintains a non-blocking TCP client connection to BeagleBone.
 *          Drains vroom bus queues and sends consolidated JSON payloads
 *          every TCP_SEND_INTERVAL_MS.
 *
 *          BeagleBone connection (outbound client):
 *          State 0 — disconnected, create socket and initiate connect
 *          State 1 — connect in progress, poll with select()
 *          State 2 — connected, ready to send
 *
 *          Motor server: delegated to motor_server.c / motor_server.h.
 *          CAM trigger:  delegated to cam_trigger.c  / cam_trigger.h.
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
 * \note    ESP32-CAM UDP trigger (2026-05-31): moved to cam_trigger.c;
 *          0->1 transition detection remains here in drain_queues().
 * \note    TempSensor BLE (2026-06-02): BLE temp sensor slot table wired in.
 *          Mirrors reed/PIR pattern. JSON "temp" field is whole degrees C
 *          matching "avg_temp" from UART/blue pill — divide by 10 at
 *          serialization point only, int16_t kept throughout.
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
#include "aws_manager.h"
#include "vroom_bus.h"
#include "trinity_log.h"
#include "pi_controller.h"
#include "motor_sm.h"
#include "pir_window.h"
#include "motor_server.h"
#include "cam_trigger.h"
#include "udp_device_ingress.h"

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

/* Per-slot previous occupied state for 0->1 transition detection */
static int g_pir_prev_occupied[MAX_PIRS] = {0};

/*----------------------------------------------------------------------------*/

typedef struct
{
   int      batt;
   uint16_t age;
   bool     active;
   uint8_t  state;
   uint8_t  offline;
   uint16_t gen;
} REED_SLOT_STATE_T;

typedef struct
{
    int      batt;
    uint32_t motion_count;
    uint16_t age;
    bool     active;
    int      occupied;
    uint8_t  offline;
} PIR_SLOT_STATE_T;

typedef struct
{
   int16_t  temp_decidegc;
   int      batt;
   uint16_t age;
   bool     active;
   uint8_t  offline;
   uint16_t gen;
} TEMP_SLOT_STATE_T;

typedef struct
{
   int               avg_temp;
   uint32_t          motion_count;
   int               pir_batt;
   int               pir_occupied;
   uint8_t           doorbell_pressed;  /* 1 = new press, cleared after send */
   uint8_t           doorbell_device_id;
   uint16_t          doorbell_age_s[MAX_DOORBELL_CAMS];  /* age per cam, 0xFFFF = never seen */
   uint16_t          cam_age_s[MAX_CAMS];       /* age per cam, 0xFFFF = never seen */
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
} TCP_STATE_T;

static TCP_STATE_T g_state =
{
    .avg_temp     = DEFAULT_AVG_TEMP,
    .pir_batt     = -1,
    .motion_count = 0,
    .pir_occupied = 0,
    .doorbell_pressed   = 0,
    .doorbell_device_id = 0,
    .lock_batt    = -1,
    .motor_batt   = -1,
    .age_pir      = 0xFFFF,
    .age_lgt      = 0xFFFF,
    .age_lck      = 0xFFFF,
    .reed_count   = 0,
    .pir_count    = 0,
    .temp_count   = 0,
};

/*----------------------------------------------------------------------------*/

static void drain_queues(BUS_SUBSCRIBER_T sub)
{
   PIR_PAYLOAD_T      p_pir;
   REED_PAYLOAD_T     p_reed;
   LOCK_PAYLOAD_T     p_lock;
   LIGHT_PAYLOAD_T    p_light;
   TEMP_PAYLOAD_T     p_temp;
   MOTOR_PAYLOAD_T    p_motor;
   BLE_TEMP_PAYLOAD_T p_ble_temp;
   int                slot        = 0;
   int                i           = 0;
   const char        *p_state_str = NULL;

   /* ---- LOCK first: latency sensitive ---- */
   if (pdTRUE == xQueueReceive(sub.mb_lock, &p_lock, 0))
   {
      g_state.lock_state = p_lock.state;
      g_state.lock_batt  = p_lock.batt;
   }

   /* ---- REED ---- */
   if (pdTRUE == xQueueReceive(sub.mb_reed, &p_reed, 0))
   {
      slot = (int)p_reed.id - 1;
      if ((0 <= slot) && (slot < MAX_REEDS))
      {
         g_state.reed_slots[slot].batt   = p_reed.batt;
         g_state.reed_slots[slot].active = true;
         g_state.reed_slots[slot].state  = p_reed.state;
      }

      if (0 == p_reed.state)      { p_state_str = "closed";  }
      else if (1 == p_reed.state) { p_state_str = "open";    }
      else                        { p_state_str = "unknown"; }

      if (1 == p_reed.id)      { ble_update_room_sensor(1, p_state_str); }
      else if (2 == p_reed.id) { ble_update_room_sensor(7, p_state_str); }
   }

   /* ---- PIR mailbox — update per-slot count and batt only ---- */
   if (pdTRUE == xQueueReceive(sub.mb_pir, &p_pir, 0))
   {
      int p_slot = (int)p_pir.id - 1;
      if ((0 <= p_slot) && (p_slot < MAX_PIRS))
      {
         g_state.pir_slots[p_slot].motion_count = p_pir.count;
         g_state.pir_slots[p_slot].batt         = p_pir.batt;
         g_state.pir_slots[p_slot].active        = true;
      }

      g_state.motion_count = 0;
      for (i = 0; i < MAX_PIRS; i++)
      {
         if (g_state.pir_slots[i].active)
         {
            g_state.motion_count += g_state.pir_slots[i].motion_count;
         }
      }

      g_state.pir_batt = p_pir.batt;
   }

   /* ---- LIGHT ---- */
   if (pdTRUE == xQueueReceive(sub.mb_light, &p_light, 0))
   {
      g_state.light_state = p_light.state;
   }

   /* ---- TEMP (UART / blue pill) ---- */
   if (pdTRUE == xQueueReceive(sub.mb_temp, &p_temp, 0))
   {
      g_state.avg_temp = p_temp.avg_temp;
   }

   /* ---- MOTOR: drain only, motor_online owned by motor_server paths ---- */
   if (pdTRUE == xQueueReceive(sub.mb_motor, &p_motor, 0))
   {
      if (p_motor.batt >= 0)
      {
         g_state.motor_batt = p_motor.batt;
      }
   }

   /* ---- BLE TEMP ---- */
   if (pdTRUE == xQueueReceive(sub.mb_ble_temp, &p_ble_temp, 0))
   {
      int t_slot = (int)p_ble_temp.id - 1;
      if ((0 <= t_slot) && (t_slot < MAX_TEMPS))
      {
         g_state.temp_slots[t_slot].temp_decidegc = p_ble_temp.temp_decidegc;
         g_state.temp_slots[t_slot].batt          = p_ble_temp.batt;
         g_state.temp_slots[t_slot].active        = true;
      }
   }

   /* ---- DOORBELL ---- */
   DOORBELL_PAYLOAD_T p_doorbell;
   if (pdTRUE == xQueueReceive(sub.mb_doorbell, &p_doorbell, 0))
   {
      g_state.doorbell_pressed   = 1;
      g_state.doorbell_device_id = p_doorbell.device_id;
   }

   /* ---- snapshot BLE state after mailboxes drained ---- */
   g_state.age_pir = ble_get_device_age_s(BLE_DEV_PIR);
   g_state.age_lgt = ble_get_device_age_s(DEV_IDX_LIGHT);
   g_state.age_lck = ble_get_device_age_s(DEV_IDX_LOCK);

   int      count      = ble_get_reed_count();
   uint16_t age        = 0xFFFF;
   uint8_t  slot_state = 0xFF;
   uint16_t gen        = 0;

   /* Sync and cache Reed array */
   for (i = 0; (i < count) && (i < MAX_REEDS); i++)
   {
      age = 0xFFFF; slot_state = 0xFF; gen = 0;
      (void)ble_get_reed_slot_info(i, NULL, NULL, &age, &slot_state, &gen);
      g_state.reed_slots[i].age     = age;
      g_state.reed_slots[i].active  = true;
      g_state.reed_slots[i].offline = (age > REED_OFFLINE_S) ? 1 : 0;
      g_state.reed_slots[i].gen     = gen;
   }

   /* Sync and cache PIR array — per-slot occupied and offline */
   g_state.pir_count = ble_pir_get_count();
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

      /* ESP32-CAM trigger on 0->1 occupancy transition */
      if (g_state.pir_slots[i].occupied && !g_pir_prev_occupied[i])
      {
         send_cam_trigger();
      }
      g_pir_prev_occupied[i] = g_state.pir_slots[i].occupied;
   }

   /* Legacy flat occupied — slot 0 for backward compat */
   g_state.pir_occupied = (g_state.pir_count > 0) ?
                           g_state.pir_slots[0].occupied : 0;

   /* Sync and cache Temp array */
   g_state.temp_count = ble_get_temp_count();
   for (i = 0; (i < g_state.temp_count) && (i < MAX_TEMPS); i++)
   {
      int16_t  t_decidegc = 0;
      int      t_batt     = -1;
      uint16_t t_age      = 0xFFFF;
      uint16_t t_gen      = 0;
      (void)ble_get_temp_slot_info(i, NULL, &t_decidegc, &t_batt,
                                   &t_age, &t_gen);
      g_state.temp_slots[i].temp_decidegc = t_decidegc;
      g_state.temp_slots[i].batt          = t_batt;
      g_state.temp_slots[i].age           = t_age;
      g_state.temp_slots[i].active        = true;
      g_state.temp_slots[i].offline       = (t_age > (TEMP_OFFLINE_MS / 1000)) ? 1 : 0;
      g_state.temp_slots[i].gen           = t_gen;
   }

   /* Snapshot doorbell liveness — mirrors ble_get_device_age_s pattern */
   for (i = 0; i < MAX_DOORBELL_CAMS; i++)
   {
      g_state.doorbell_age_s[i] = doorbell_get_age_s((uint8_t)i);
   }

   /* Snapshot camera liveness — mirrors doorbell pattern */
   for (i = 0; i < MAX_CAMS; i++)
   {
      g_state.cam_age_s[i] = cam_get_age_s((uint8_t)i);
   }

   /* ---- Cam debug log ---- */
   for (i = 0; i < MAX_CAMS; i++)
   {
      ESP_LOGI(TAG, "[CAM] slot=%d age_s=%d online=%d",
               i,
               g_state.cam_age_s[i],
               cam_is_alive((uint8_t)i) ? 1 : 0);
   }

   /* ---- Doorbell debug log ---- */
   for (i = 0; i < MAX_DOORBELL_CAMS; i++)
   {
      ESP_LOGI(TAG, "[DOORBELL] id=%d age_s=%d online=%d",
               i,
               g_state.doorbell_age_s[i],
               doorbell_is_alive((uint8_t)i) ? 1 : 0);
   }

   /* ---- Reed debug log ---- */
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

   /* ---- PIR debug log ---- */
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

   /* ---- Temp debug log ---- */
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
   cJSON   *p_root  = NULL;
   cJSON   *p_reeds = NULL;
   cJSON   *p_rooms = NULL;
   cJSON   *p_entry = NULL;
   cJSON   *p_pirs  = NULL;
   cJSON   *p_temps = NULL;
   char    *p_msg   = NULL;
   char     name[REED_NAME_BUF_SIZE] = {0};
   uint8_t  door_state = 0xFF;
   uint16_t gen        = 0;
   int      i          = 0;
   int      sent       = 0;

   if ((TCP_STATE_CONNECTED != *p_bb_state) || (SOCK_INVALID == *p_bb_sock)) { return; }

   p_root = cJSON_CreateObject();
   if (NULL == p_root) { ESP_LOGE(TAG, "cJSON root alloc failed (BB)"); return; }

   (void)cJSON_AddNumberToObject(p_root, "avg_temp",     g_state.avg_temp);
   (void)cJSON_AddNumberToObject(p_root, "motion_count", g_state.motion_count);
   (void)cJSON_AddNumberToObject(p_root, "light_state",  g_state.light_state);
   (void)cJSON_AddNumberToObject(p_root, "lock_state",   g_state.lock_state);
   (void)cJSON_AddNumberToObject(p_root, "age_pir",      g_state.age_pir);
   (void)cJSON_AddNumberToObject(p_root, "age_lgt",      g_state.age_lgt);
   (void)cJSON_AddNumberToObject(p_root, "age_lck",      g_state.age_lck);
   (void)cJSON_AddNumberToObject(p_root, "batt_pir",     g_state.pir_batt);
   (void)cJSON_AddNumberToObject(p_root, "pir_occupied", g_state.pir_occupied);
   (void)cJSON_AddNumberToObject(p_root, "doorbell_pressed",   g_state.doorbell_pressed);
   (void)cJSON_AddNumberToObject(p_root, "doorbell_device_id", g_state.doorbell_device_id);
   (void)cJSON_AddNumberToObject(p_root, "batt_lck",     g_state.lock_batt);
   (void)cJSON_AddNumberToObject(p_root, "batt_motor",   g_state.motor_batt);
   (void)cJSON_AddNumberToObject(p_root, "motor_online", g_state.motor_online);

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
      (void)cJSON_AddItemToObject(p_root, "reeds", p_reeds);
   }

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
      (void)cJSON_AddItemToObject(p_root, "rooms", p_rooms);
   }

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
      (void)cJSON_AddItemToObject(p_root, "pirs", p_pirs);
   }
   (void)cJSON_AddNumberToObject(p_root, "pir_count", g_state.pir_count);

   /* temps — "temp" field is whole degrees C, matches avg_temp convention */
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
            (void)cJSON_AddNumberToObject(p_entry, "id",
                                          i + 1);
            (void)cJSON_AddNumberToObject(p_entry, "temp",
                                          g_state.temp_slots[i].temp_decidegc / 10);
            (void)cJSON_AddNumberToObject(p_entry, "batt",
                                          g_state.temp_slots[i].batt);
            (void)cJSON_AddNumberToObject(p_entry, "age",
                                          g_state.temp_slots[i].age);
            (void)cJSON_AddNumberToObject(p_entry, "offline",
                                          g_state.temp_slots[i].offline);
            (void)cJSON_AddNumberToObject(p_entry, "gen",    t_gen);
            (void)cJSON_AddStringToObject(p_entry, "name",   t_name);
            (void)cJSON_AddItemToArray(p_temps, p_entry);
         }
      }
      (void)cJSON_AddItemToObject(p_root, "temps", p_temps);
   }
   (void)cJSON_AddNumberToObject(p_root, "temp_count", g_state.temp_count);

   cJSON *p_cams = cJSON_CreateArray();
   if (NULL != p_cams)
   {
      for (i = 0; i < MAX_CAMS; i++)
      {
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddNumberToObject(p_entry, "id",     i);
            (void)cJSON_AddNumberToObject(p_entry, "age_s",  g_state.cam_age_s[i]);
            (void)cJSON_AddNumberToObject(p_entry, "online", cam_is_alive((uint8_t)i) ? 1 : 0);
            (void)cJSON_AddItemToArray(p_cams, p_entry);
         }
      }
      (void)cJSON_AddItemToObject(p_root, "cams", p_cams);
   }

   cJSON *p_doorbells = cJSON_CreateArray();
   if (NULL != p_doorbells)
   {
      for (i = 0; i < MAX_DOORBELL_CAMS; i++)
      {
          p_entry = cJSON_CreateObject();
          if (NULL != p_entry)
            {
               (void)cJSON_AddNumberToObject(p_entry, "id",      i);
               (void)cJSON_AddNumberToObject(p_entry, "age_s",   g_state.doorbell_age_s[i]);
               (void)cJSON_AddNumberToObject(p_entry, "online",  doorbell_is_alive((uint8_t)i) ? 1 : 0);
               (void)cJSON_AddItemToArray(p_doorbells, p_entry);
            }
      }
    (void)cJSON_AddItemToObject(p_root, "doorbells", p_doorbells);
}

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

      ESP_LOGI(TAG, "[BEAGLEBONE] tmp=%d pir=%u occ=%d lgt=%d lck=%d "
                    "reeds=%d temps=%d mtr=%d batt_mtr=%d batt_pir=%d batt_lck=%d" 
                    " doorbell=%d db_id=%d",
               g_state.avg_temp, (unsigned)g_state.motion_count,
               g_state.pir_occupied,
               g_state.light_state, g_state.lock_state, g_state.reed_count,
               g_state.temp_count,
               (int)g_state.motor_online, g_state.motor_batt,
               g_state.pir_batt, g_state.lock_batt,
               g_state.doorbell_pressed, g_state.doorbell_device_id);

      for (i = 0; i < g_state.pir_count; i++)
      {
         ESP_LOGI(TAG, "[PIR_SEND] slot=%d count=%u batt=%d age=%d offline=%d occ=%d",
                  i + 1,
                  (unsigned)g_state.pir_slots[i].motion_count,
                  g_state.pir_slots[i].batt,
                  g_state.pir_slots[i].age,
                  g_state.pir_slots[i].offline,
                  g_state.pir_slots[i].occupied);
      }
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
         ESP_LOGI(TAG, "[BEAGLEBONE] Connecting to %s:%d", BEAGLEBONE_IP, BEAGLEBONE_PORT);
         (void)fcntl(*p_sock, F_SETFL, flags | O_NONBLOCK);
         ret = connect(*p_sock, (struct sockaddr *)p_addr, sizeof(*p_addr));
         if (0 == ret)
         {
            trinity_log_event("EVENT: TCP_BB_CONNECTED\n");
            *p_state = TCP_STATE_CONNECTED; *p_block_count = 0;
            ESP_LOGI(TAG, "[BEAGLEBONE] Connected (immediate)");
         }
         else if (EINPROGRESS == errno)
         {
            ESP_LOGI(TAG, "[BEAGLEBONE] Connect in progress...");
            *p_connect_start = now; *p_state = TCP_STATE_CONNECTING;
         }
         else
         {
            ESP_LOGW(TAG, "[BEAGLEBONE] Connect failed (errno=%d), retrying", errno);
            close(*p_sock); *p_sock = SOCK_INVALID;
            vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS));
         }
         break;
      }
      case TCP_STATE_CONNECTING:
      {
         if ((now - *p_connect_start) > BB_CONNECT_TIMEOUT_MS)
         {
            ESP_LOGW(TAG, "[BEAGLEBONE] Connect timed out after %d ms", BB_CONNECT_TIMEOUT_MS);
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
               ESP_LOGW(TAG, "[BEAGLEBONE] Connect error (SO_ERROR=%d)", err);
               close(*p_sock); *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED;
               vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS));
            }
            else if (FD_ISSET(*p_sock, &writefds))
            {
               trinity_log_event("EVENT: TCP_BB_CONNECTED\n");
               ESP_LOGI(TAG, "[BEAGLEBONE] Connected");
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
      drain_queues(sub);

      send_to_bb(&bb_sock, &bb_block_count, &bb_state);
   }
}
