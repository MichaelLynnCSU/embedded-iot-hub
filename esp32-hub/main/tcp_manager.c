/******************************************************************************
 * \file tcp_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief TCP manager for ESP32 hub node.
 *
 * \details Maintains non-blocking TCP connections to BeagleBone and
 *          ESP32-C3 motor controller. Drains vroom bus queues and sends
 *          consolidated JSON payloads every TCP_SEND_INTERVAL_MS.
 *
 *          Connection state machine (per socket):
 *          State 0 — disconnected, create socket and initiate connect
 *          State 1 — connect in progress, poll with select()
 *          State 2 — connected, ready to send
 *
 *          Backpressure handling:
 *          - EAGAIN/EWOULDBLOCK increments block counter
 *          - 5 consecutive blocks triggers force reconnect
 *
 *          Reed sensors emitted as dynamic JSON array — adding more
 *          ReedSensor nodes requires zero changes here or on BeagleBone.
 *
 * \note    Phantom widget fix (2026-04-21):
 *          Re-querying ble_get_reed_count() in build_and_send() creates a TOCTOU
 *          race -- a BLE advertisement arriving between drain_queues() and
 *          build_and_send() can change the slot count, causing the JSON array to
 *          shrink and the dashboard to remove a widget for 1-2 seconds.
 *          Fix: snapshot count once here and use g_state.reed_count in
 *          build_and_send() so both the slot data and array length come from the
 *          same moment in time.
 *
 * \note    WDT fix (2026-03-21):
 *          trinity_wdt_kick() added at the top of the main while(1) loop.
 *          The event group wait uses TCP_SEND_INTERVAL_MS as its timeout,
 *          which must be less than the WDT timeout (5 s). Verify
 *          TCP_SEND_INTERVAL_MS in config.h is < 5000.
 *
 * \note    Motor battery (2026-04-08, updated 2026-04-27):
 *          TCP_STATE_T gains motor_batt field (SOC percent, -1 = unknown).
 *          Motor node previously sent raw millivolts and hub converted with
 *          motor_mv_to_percent(). Motor now sends SOC percent directly:
 *          {"batt_motor": <percent>}. motor_mv_to_percent() removed.
 *          Hub stores and forwards percent unchanged -- no hardcoded
 *          voltage thresholds or math on the hub side.
 *
 * \note    BB logging fix (2026-04-27):
 *          run_bb_state_machine() was missing ESP_LOGI calls present in
 *          run_c3_state_machine(), making it impossible to tell from logs
 *          whether the BeagleBone connection succeeded or failed.
 *          Added: connecting attempt log, immediate-connect log, EINPROGRESS
 *          log, connected-via-select log, and timeout log.
 *          Also fixed build_and_send() where the [BEAGLEBONE] success log
 *          printed local variable `count` (always 0) instead of
 *          g_state.reed_count.
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
#include "uart_manager.h"
#include "tcp_manager.h"
#include "vroom_bus.h"
#include "trinity_log.h"

#define MAX_REEDS               6         /**< must match ble_scan.c and BeagleBone */
#define REED_OFFLINE_S          150       /**< reed offline threshold seconds */
#define BB_CONNECT_TIMEOUT_MS   2000      /**< BeagleBone connect timeout ms */
#define C3_CONNECT_TIMEOUT_MS   10000     /**< C3 motor connect timeout ms */
#define BLOCK_COUNT_MAX         5         /**< max consecutive send blocks */
#define SOCK_POLL_DELAY_MS      100       /**< delay when no connections ready ms */
#define SOCK_RETRY_DELAY_MS     1000      /**< socket creation retry delay ms */
#define REED_NAME_BUF_SIZE      32        /**< reed slot name buffer size */
#define SOCK_INVALID            -1        /**< sentinel for invalid socket fd */
#define TCP_STATE_DISCONNECTED  0         /**< socket state: disconnected */
#define TCP_STATE_CONNECTING    1         /**< socket state: connect in progress */
#define TCP_STATE_CONNECTED     2         /**< socket state: connected */

static const char *TAG = "TCP_MGR"; /**< ESP log tag */

/** \brief Per-reed slot state snapshot for TCP JSON payload */
typedef struct
{
   int      batt;    /*!< battery SOC percent */
   uint16_t age;     /*!< seconds since last advertisement */
   bool     active;  /*!< slot has been seen at least once */
   uint8_t  state;   /*!< 0=closed 1=open 0xFF=unknown */
   uint8_t  offline; /*!< 1=SLOT_OFFLINE, tile visible with red dot */
   uint16_t gen;     /*!< generation counter, increments on device swap */
} REED_SLOT_STATE_T;

/** \brief Local snapshot of all sensor state for TCP JSON payload */
typedef struct
{
   int              avg_temp;               /*!< average temperature from STM32 */
   uint32_t         motion_count;           /*!< PIR motion event count */
   int              pir_batt;               /*!< PIR battery SOC percent */
   int              pir_occupied;           /* 0=empty, 1=occupied  -- new */
   uint8_t          light_state;            /*!< smart light relay state */
   uint8_t          lock_state;             /*!< smart lock state */
   int              lock_batt;              /*!< smart lock battery SOC */
   uint8_t          motor_online;           /*!< C3 motor controller online flag */
   int              motor_batt;             /*!< motor battery SOC percent, -1=unknown */
   REED_SLOT_STATE_T reed_slots[MAX_REEDS]; /*!< per-reed slot state */
   uint16_t         age_pir;               /*!< PIR device age seconds */
   uint16_t         age_lgt;               /*!< light device age seconds */
   uint16_t         age_lck;               /*!< lock device age seconds */
   int              reed_count;            /*!< snapshotted reed count -- see phantom widget fix note */
} TCP_STATE_T;

static TCP_STATE_T g_state =
{
   .avg_temp   = DEFAULT_AVG_TEMP,
   .pir_batt   = -1,
   .lock_batt  = -1,
   .motor_batt = -1,
   .age_pir    = 0xFFFF,
   .age_lgt    = 0xFFFF,
   .age_lck    = 0xFFFF,
};

static void drain_queues(EventBits_t bits)
{
   PIR_PAYLOAD_T   p_pir;
   REED_PAYLOAD_T  p_reed;
   LOCK_PAYLOAD_T  p_lock;
   LIGHT_PAYLOAD_T p_light;
   TEMP_PAYLOAD_T  p_temp;
   MOTOR_PAYLOAD_T p_motor;
   int             slot        = 0;
   int             count       = 0;
   int             i           = 0;
   uint16_t        age         = 0xFFFF;
   uint8_t         slot_state  = 0xFF;
   uint16_t        gen         = 0;
   const char     *p_state_str = NULL;

   if (0 != (bits & EVT_BLE_PIR))
   {
      while (pdTRUE == xQueueReceive(q_pir, &p_pir, 0))
      {
         g_state.motion_count = p_pir.count;
         g_state.pir_batt     = p_pir.batt;
      }
      g_state.pir_occupied = ble_get_pir_occupied();
   }

   if (0 != (bits & EVT_BLE_REED))
   {
      while (pdTRUE == xQueueReceive(q_reed, &p_reed, 0))
      {
         slot = (int)p_reed.id - 1;

         if ((0 <= slot) && (slot < MAX_REEDS))
         {
            g_state.reed_slots[slot].batt   = p_reed.batt;
            g_state.reed_slots[slot].active = true;
            g_state.reed_slots[slot].state  = p_reed.state;
         }

         if (0 == p_reed.state)       { p_state_str = "closed";  }
         else if (1 == p_reed.state)  { p_state_str = "open";    }
         else                         { p_state_str = "unknown"; }

         if (1 == p_reed.id)       { ble_update_room_sensor(1, p_state_str); }
         else if (2 == p_reed.id)  { ble_update_room_sensor(7, p_state_str); }
      }
   }

   if (0 != (bits & EVT_BLE_LOCK))
   {
      while (pdTRUE == xQueueReceive(q_lock, &p_lock, 0))
      {
         g_state.lock_state = p_lock.state;
         g_state.lock_batt  = p_lock.batt;
      }
   }

   if (0 != (bits & EVT_BLE_LIGHT))
   {
      while (pdTRUE == xQueueReceive(q_light, &p_light, 0))
      {
         g_state.light_state = p_light.state;
      }
   }

   if (0 != (bits & EVT_UART_TEMP))
   {
      while (pdTRUE == xQueueReceive(q_temp, &p_temp, 0))
      {
         g_state.avg_temp = p_temp.avg_temp;
      }
   }

   if (0 != (bits & EVT_MOTOR_STATUS))
   {
      while (pdTRUE == xQueueReceive(q_motor, &p_motor, 0))
      {
         g_state.motor_online = p_motor.online;
         /* Only update batt when online -- preserve last known on disconnect.
          * Motor sends SOC percent directly -- no conversion needed. */
         if (p_motor.online && (p_motor.batt >= 0))
         {
            g_state.motor_batt = p_motor.batt;
         }
      }
   }

   g_state.age_pir = ble_get_device_age_s(BLE_DEV_PIR);
   g_state.age_lgt = ble_get_device_age_s(BLE_DEV_LIGHT);
   g_state.age_lck = ble_get_device_age_s(BLE_DEV_LOCK);

   count = ble_get_reed_count();

   for (i = 0; (i < count) && (i < MAX_REEDS); i++)
   {
      age        = 0xFFFF;
      slot_state = 0xFF;
      gen        = 0;

      (void)ble_get_reed_slot_info(i, NULL, NULL, &age, &slot_state, &gen);
      g_state.reed_slots[i].age     = age;
      g_state.reed_slots[i].active  = true;
      g_state.reed_slots[i].offline = (age > REED_OFFLINE_S) ? 1 : 0;
      g_state.reed_slots[i].gen     = gen;
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
}

static void handle_c3_send_error(int *p_sock,
                                  int *p_block_count,
                                  int *p_state)
{
   if ((EAGAIN == errno) || (EWOULDBLOCK == errno))
   {
      (*p_block_count)++;
      ESP_LOGW(TAG, "[C3_MOTOR] Send would block (%d)", *p_block_count);

      if (*p_block_count >= BLOCK_COUNT_MAX)
      {
         trinity_log_event("EVENT: TCP_C3_FORCE_RECONNECT\n");
         close(*p_sock);
         *p_sock        = SOCK_INVALID;
         *p_state       = TCP_STATE_DISCONNECTED;
         *p_block_count = 0;
         g_state.motor_online = 0;
         bus_publish_motor(0, -1);
      }
   }
   else
   {
      trinity_log_event("EVENT: TCP_C3_DISCONNECTED\n");
      close(*p_sock);
      *p_sock        = SOCK_INVALID;
      *p_state       = TCP_STATE_DISCONNECTED;
      *p_block_count = 0;
      bus_publish_motor(0, -1);
      g_state.motor_online = 0;
   }
}

static void handle_bb_send_error(int *p_sock,
                                  int *p_block_count,
                                  int *p_state)
{
   if ((EAGAIN == errno) || (EWOULDBLOCK == errno))
   {
      (*p_block_count)++;
      ESP_LOGW(TAG, "[BEAGLEBONE] Send would block (%d)", *p_block_count);

      if (*p_block_count >= BLOCK_COUNT_MAX)
      {
         trinity_log_event("EVENT: TCP_BB_FORCE_RECONNECT\n");
         close(*p_sock);
         *p_sock        = SOCK_INVALID;
         *p_state       = TCP_STATE_DISCONNECTED;
         *p_block_count = 0;
      }
   }
   else
   {
      trinity_log_event("EVENT: TCP_BB_DISCONNECTED\n");
      close(*p_sock);
      *p_sock        = SOCK_INVALID;
      *p_state       = TCP_STATE_DISCONNECTED;
      *p_block_count = 0;
   }
}

static void build_and_send(int *p_bb_sock,
                            int *p_c3_sock,
                            int *p_bb_block_count,
                            int *p_c3_block_count,
                            int *p_bb_state,
                            int *p_c3_state)
{
   cJSON   *p_root     = NULL;
   cJSON   *p_reeds    = NULL;
   cJSON   *p_rooms    = NULL;
   cJSON   *p_entry    = NULL;
   cJSON   *p_rx_json  = NULL;
   cJSON   *p_batt     = NULL;
   char    *p_msg      = NULL;
   char     name[REED_NAME_BUF_SIZE] = {0};
   char     rx[64]     = {0};
   uint8_t  door_state = 0xFF;
   uint16_t gen        = 0;
   int      i          = 0;
   int      sent       = 0;
   int      rlen       = 0;

   p_root = cJSON_CreateObject();
   if (NULL == p_root)
   {
      ESP_LOGE(TAG, "cJSON root alloc failed");
      return;
   }

   (void)cJSON_AddNumberToObject(p_root, "avg_temp",     g_state.avg_temp);
   (void)cJSON_AddNumberToObject(p_root, "motion_count", g_state.motion_count);
   (void)cJSON_AddNumberToObject(p_root, "light_state",  g_state.light_state);
   (void)cJSON_AddNumberToObject(p_root, "lock_state",   g_state.lock_state);
   (void)cJSON_AddNumberToObject(p_root, "age_pir",      g_state.age_pir);
   (void)cJSON_AddNumberToObject(p_root, "age_lgt",      g_state.age_lgt);
   (void)cJSON_AddNumberToObject(p_root, "age_lck",      g_state.age_lck);
   (void)cJSON_AddNumberToObject(p_root, "batt_pir",     g_state.pir_batt);
   (void)cJSON_AddNumberToObject(p_root, "pir_occupied",  g_state.pir_occupied);
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
         door_state = 0xFF;
         gen        = 0;

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

   p_msg = cJSON_PrintUnformatted(p_root);
   cJSON_Delete(p_root);

   if (NULL == p_msg)
   {
      ESP_LOGE(TAG, "cJSON serialize failed");
      return;
   }

   if ((TCP_STATE_CONNECTED == *p_c3_state) &&
       (SOCK_INVALID != *p_c3_sock))
   {
      sent = send(*p_c3_sock, p_msg, strlen(p_msg), 0);
      if (0 > sent)
      {
         handle_c3_send_error(p_c3_sock, p_c3_block_count, p_c3_state);
      }
      else
      {
         *p_c3_block_count = 0;
         ESP_LOGI(TAG, "[C3_MOTOR] Sent %d bytes", sent);
         g_state.motor_online = 1;

         /* Drain battery response from motor -- arrives as SOC percent.
          * {"batt_motor": <percent>} -- store directly, no conversion. */
         rlen = recv(*p_c3_sock, rx, sizeof(rx) - 1, MSG_DONTWAIT);
         if (rlen > 0)
         {
            rx[rlen]  = '\0';
            p_rx_json = cJSON_Parse(rx);
            if (NULL != p_rx_json)
            {
               p_batt = cJSON_GetObjectItem(p_rx_json, "batt_motor");
               if ((NULL != p_batt) && (p_batt->valueint >= 0))
               {
                  g_state.motor_batt = p_batt->valueint;
                  ESP_LOGI(TAG, "[C3_MOTOR] batt_motor=%d%%", g_state.motor_batt);
               }
               cJSON_Delete(p_rx_json);
            }
         }
      }
   }

   if ((TCP_STATE_CONNECTED == *p_bb_state) &&
       (SOCK_INVALID != *p_bb_sock))
   {
      sent = send(*p_bb_sock, p_msg, strlen(p_msg), 0);
      if (0 > sent)
      {
         handle_bb_send_error(p_bb_sock, p_bb_block_count, p_bb_state);
      }
      else
      {
         *p_bb_block_count = 0;
         ESP_LOGI(TAG, "[BEAGLEBONE] tmp=%d pir=%u occ=%d lgt=%d lck=%d reeds=%d mtr=%d batt_mtr=%d batt_pir=%d batt_lck=%d",
                  g_state.avg_temp,
                  (unsigned)g_state.motion_count,
                  g_state.pir_occupied,
                  g_state.light_state,
                  g_state.lock_state,
                  g_state.reed_count,
                  (int)g_state.motor_online,
                  g_state.motor_batt,
                  g_state.pir_batt,
                  g_state.lock_batt);
      }
   }

   cJSON_free(p_msg);
}

static void run_c3_state_machine(int *p_sock,
                                  int *p_state,
                                  int *p_block_count,
                                  uint32_t *p_connect_start,
                                  struct sockaddr_in *p_addr,
                                  uint32_t now)
{
   int            flags = 0;
   int            ret   = 0;
   int            err   = 0;
   socklen_t      el    = sizeof(err);
   fd_set         writefds;
   fd_set         errorfds;
   struct timeval tv;

   switch (*p_state)
   {
      case TCP_STATE_DISCONNECTED:
      {
         if (SOCK_INVALID != *p_sock) { close(*p_sock); *p_sock = SOCK_INVALID; }

         *p_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
         if (0 > *p_sock) { vTaskDelay(pdMS_TO_TICKS(SOCK_RETRY_DELAY_MS)); break; }

         flags = fcntl(*p_sock, F_GETFL, 0);
         ESP_LOGI(TAG, "[C3_MOTOR] Connecting to %s:%d", C3_MOTOR_IP, C3_MOTOR_PORT);
         (void)fcntl(*p_sock, F_SETFL, flags | O_NONBLOCK);

         ret = connect(*p_sock, (struct sockaddr *)p_addr, sizeof(*p_addr));

         if (0 == ret)
         {
            *p_state = TCP_STATE_CONNECTED; *p_block_count = 0;
            ESP_LOGI(TAG, "[C3_MOTOR] Connected");
            bus_publish_motor(1, -1);
         }
         else if (EINPROGRESS == errno)
         {
            *p_connect_start = now;
            *p_state         = TCP_STATE_CONNECTING;
         }
         else
         {
            close(*p_sock); *p_sock = SOCK_INVALID;
            vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS));
         }
         break;
      }

      case TCP_STATE_CONNECTING:
      {
         if ((now - *p_connect_start) > C3_CONNECT_TIMEOUT_MS)
         {
            close(*p_sock); *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED;
            vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS));
            break;
         }

         FD_ZERO(&writefds); FD_ZERO(&errorfds);
         FD_SET(*p_sock, &writefds); FD_SET(*p_sock, &errorfds);
         tv.tv_sec = 0; tv.tv_usec = 10000;

         if (0 < select(*p_sock + 1, NULL, &writefds, &errorfds, &tv))
         {
            (void)getsockopt(*p_sock, SOL_SOCKET, SO_ERROR, &err, &el);

            if (FD_ISSET(*p_sock, &errorfds) || (0 != err))
            {
               close(*p_sock); *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED;
               vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS));
            }
            else if (FD_ISSET(*p_sock, &writefds))
            {
               trinity_log_event("EVENT: TCP_C3_CONNECTED\n");
               *p_state = TCP_STATE_CONNECTED; *p_block_count = 0;
               bus_publish_motor(1, -1);
            }
         }
         break;
      }

      case TCP_STATE_CONNECTED:  break;
      default: *p_state = TCP_STATE_DISCONNECTED; break;
   }
}

static void run_bb_state_machine(int *p_sock,
                                  int *p_state,
                                  int *p_block_count,
                                  uint32_t *p_connect_start,
                                  struct sockaddr_in *p_addr,
                                  uint32_t now)
{
   int            flags = 0;
   int            ret   = 0;
   int            err   = 0;
   socklen_t      el    = sizeof(err);
   fd_set         writefds;
   fd_set         errorfds;
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
            vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS));
            break;
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

      case TCP_STATE_CONNECTED:  break;
      default: *p_state = TCP_STATE_DISCONNECTED; break;
   }
}

void tcp_manager_init(void)
{
   ESP_LOGI(TAG, "TCP manager initialized");
}

void tcp_manager_task(EventGroupHandle_t p_system_eg,
                      EventGroupHandle_t p_wifi_eg,
                      EventGroupHandle_t p_events)
{
   int      c3_sock          = SOCK_INVALID;
   int      bb_sock          = SOCK_INVALID;
   int      c3_state         = TCP_STATE_DISCONNECTED;
   int      bb_state         = TCP_STATE_DISCONNECTED;
   int      c3_block_count   = 0;
   int      bb_block_count   = 0;
   uint32_t c3_connect_start = 0;
   uint32_t bb_connect_start = 0;
   uint32_t now              = 0;
   EventBits_t bits          = 0;

   (void)p_system_eg;

   struct sockaddr_in c3_addr =
   {
      .sin_family      = AF_INET,
      .sin_port        = htons(C3_MOTOR_PORT),
      .sin_addr.s_addr = inet_addr(C3_MOTOR_IP),
   };

   struct sockaddr_in bb_addr =
   {
      .sin_family      = AF_INET,
      .sin_port        = htons(BEAGLEBONE_PORT),
      .sin_addr.s_addr = inet_addr(BEAGLEBONE_IP),
   };

   (void)xEventGroupWaitBits(p_wifi_eg, WIFI_CONNECTED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);
   ESP_LOGI(TAG, "TCP task running");

   while (1)
   {
      trinity_wdt_kick();

      now = xTaskGetTickCount() * portTICK_PERIOD_MS;

      run_c3_state_machine(&c3_sock, &c3_state, &c3_block_count,
                            &c3_connect_start, &c3_addr, now);

      run_bb_state_machine(&bb_sock, &bb_state, &bb_block_count,
                            &bb_connect_start, &bb_addr, now);

      if ((TCP_STATE_CONNECTED != c3_state) &&
          (TCP_STATE_CONNECTED != bb_state))
      {
         vTaskDelay(pdMS_TO_TICKS(SOCK_POLL_DELAY_MS));
         continue;
      }

      bits = xEventGroupWaitBits(p_events,
                                  EVT_ALL_MASK,
                                  pdTRUE,
                                  pdFALSE,
                                  pdMS_TO_TICKS(TCP_SEND_INTERVAL_MS));

      drain_queues(bits);

      g_state.motor_online = (TCP_STATE_CONNECTED == c3_state) ? 1 : 0;
      build_and_send(&bb_sock, &c3_sock,
                     &bb_block_count, &c3_block_count,
                     &bb_state, &c3_state);
   }
}
