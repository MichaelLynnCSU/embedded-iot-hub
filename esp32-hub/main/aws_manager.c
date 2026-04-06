/******************************************************************************
 * \file aws_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief AWS IoT Lambda manager for ESP32 hub node.
 *
 * \details Maintains a local snapshot of all BLE and UART sensor state
 *          by draining the vroom bus queues on every wakeup. Sends a
 *          consolidated JSON payload to AWS Lambda every 5 minutes.
 *          Parses the Lambda response for motor/light control updates.
 *
 *          Send interval: AWS_SEND_INTERVAL_MS (5 minutes)
 *          Drain interval: 2 seconds (keeps state fresh between sends,
 *          and doubles as the WDT kick interval)
 *
 *          JSON payload fields:
 *          avg_temp, motion_count, light_state, lock_state,
 *          batt_pir, batt_dr1, batt_dr2, batt_lck, motor_online, rooms[]
 *
 * \note    WDT fix (2026-03-21):
 *          DRAIN_INTERVAL_MS reduced from 10000 ms to 2000 ms so the
 *          xEventGroupWaitBits timeout is well within the 5 s WDT window.
 *          trinity_wdt_kick() added at the top of the main while(1) loop.
 *          trinity_log_event() renamed to trinity_log_event() throughout.
 ******************************************************************************/

#include "config.h"
#include "network_config.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include "ble_manager.h"
#include "uart_manager.h"
#include "aws_manager.h"
#include "vroom_bus.h"
#include "trinity_log.h"

#define DRAIN_INTERVAL_MS   2000   /**< queue drain / WDT kick interval ms (must be < WDT timeout) */
#define HTTP_TIMEOUT_MS     10000  /**< HTTP request timeout ms */
#define HTTP_STATUS_OK      200    /**< expected HTTP success status */

static const char *TAG = "AWS_MGR"; /**< ESP log tag */

static int g_aws_low   = DEFAULT_AWS_LOW;   /**< motor low temp threshold */
static int g_aws_high  = DEFAULT_AWS_HIGH;  /**< motor high temp threshold */
static int g_aws_motor = DEFAULT_AWS_MOTOR; /**< motor control mode */

static char g_response_buffer[HTTP_RESPONSE_BUFFER_SIZE]; /**< HTTP response buf */
static int  g_response_len = 0;                           /**< response byte count */

/** \brief Local snapshot of all sensor state — updated by drain_queues() */
typedef struct
{
   int      avg_temp;      /*!< average temperature from STM32 */
   uint32_t motion_count;  /*!< PIR motion event count */
   int      pir_batt;      /*!< PIR sensor battery SOC */
   uint8_t  light_state;   /*!< smart light relay state */
   uint8_t  lock_state;    /*!< smart lock state */
   int      lock_batt;     /*!< smart lock battery SOC */
   int      dr1_batt;      /*!< door reed 1 battery SOC */
   int      dr2_batt;      /*!< door reed 2 battery SOC */
   uint8_t  motor_online;  /*!< C3 motor controller online flag */
} AWS_STATE_T;

static AWS_STATE_T g_state =
{
   .avg_temp  = DEFAULT_AVG_TEMP,
   .pir_batt  = -1,
   .lock_batt = -1,
   .dr1_batt  = -1,
   .dr2_batt  = -1,
};

static void drain_queues(EventBits_t bits)
{
   PIR_PAYLOAD_T   p_pir;
   REED_PAYLOAD_T  p_reed;
   LOCK_PAYLOAD_T  p_lock;
   LIGHT_PAYLOAD_T p_light;
   TEMP_PAYLOAD_T  p_temp;
   MOTOR_PAYLOAD_T p_motor;

   if (0 != (bits & EVT_BLE_PIR))
   {
      while (pdTRUE == xQueueReceive(q_pir, &p_pir, 0))
      {
         g_state.motion_count = p_pir.count;
         g_state.pir_batt     = p_pir.batt;
      }
   }

   if (0 != (bits & EVT_BLE_REED))
   {
      while (pdTRUE == xQueueReceive(q_reed, &p_reed, 0))
      {
         if (1 == p_reed.id) { g_state.dr1_batt = p_reed.batt; }
         else                { g_state.dr2_batt = p_reed.batt; }
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
      }
   }
}

static esp_err_t http_event_handler(esp_http_client_event_t *p_evt)
{
   if ((HTTP_EVENT_ON_DATA == p_evt->event_id) &&
       (0 != p_evt->data_len))
   {
      if ((g_response_len + p_evt->data_len) < (int)sizeof(g_response_buffer))
      {
         (void)memcpy(g_response_buffer + g_response_len,
                      p_evt->data,
                      p_evt->data_len);
         g_response_len += p_evt->data_len;
         g_response_buffer[g_response_len] = 0;
      }
   }

   return ESP_OK;
}

static bool send_to_aws(const char *p_post_data)
{
   esp_err_t err    = ESP_OK;
   int       status = 0;
   bool      success = false;
   esp_http_client_handle_t client;

   g_response_len = 0;
   (void)memset(g_response_buffer, 0, sizeof(g_response_buffer));

   esp_http_client_config_t config =
   {
      .url               = AWS_LAMBDA_URL,
      .method            = HTTP_METHOD_POST,
      .event_handler     = http_event_handler,
      .timeout_ms        = HTTP_TIMEOUT_MS,
      .transport_type    = HTTP_TRANSPORT_OVER_SSL,
      .crt_bundle_attach = esp_crt_bundle_attach,
   };

   client = esp_http_client_init(&config);
   if (NULL == client)
   {
      ESP_LOGE(TAG, "Failed to init HTTP client");
      trinity_log_event("EVENT: AWS_HTTP_INIT_FAIL\n");
      return false;
   }

   (void)esp_http_client_set_header(client, "Content-Type", "application/json");
   (void)esp_http_client_set_post_field(client, p_post_data, strlen(p_post_data));

   ESP_LOGI(TAG, "Sending to AWS...");
   err = esp_http_client_perform(client);

   if (ESP_OK == err)
   {
      status = esp_http_client_get_status_code(client);
      ESP_LOGI(TAG, "AWS HTTP Status = %d", status);

      if (HTTP_STATUS_OK == status)
      {
         if (0 < g_response_len)
         {
            ESP_LOGI(TAG, "AWS Response: %s", g_response_buffer);
         }
         success = true;
      }
   }
   else
   {
      ESP_LOGE(TAG, "AWS request failed: %s", esp_err_to_name(err));
      trinity_log_event("EVENT: AWS_REQUEST_FAIL\n");
   }

   (void)esp_http_client_cleanup(client);

   return success;
}

static void parse_control_response(void)
{
   cJSON *p_json  = NULL;
   cJSON *p_low   = NULL;
   cJSON *p_high  = NULL;
   cJSON *p_motor = NULL;
   cJSON *p_light = NULL;

   if (0 == g_response_len) { return; }

   p_json = cJSON_Parse(g_response_buffer);
   if (NULL == p_json) { return; }

   p_low   = cJSON_GetObjectItem(p_json, "low");
   p_high  = cJSON_GetObjectItem(p_json, "high");
   p_motor = cJSON_GetObjectItem(p_json, "motor");
   p_light = cJSON_GetObjectItem(p_json, "light");

   if (NULL != p_low)   { g_aws_low   = p_low->valueint;  }
   if (NULL != p_high)  { g_aws_high  = p_high->valueint; }
   if (NULL != p_motor) { g_aws_motor = p_motor->valueint; }
   if (NULL != p_light) { ble_send_light_command(p_light->valueint); }

   ESP_LOGI(TAG, "Control updated: LOW=%d HIGH=%d MOTOR=%d",
            g_aws_low, g_aws_high, g_aws_motor);

   cJSON_Delete(p_json);
}

static void send_state_to_aws(void)
{
   cJSON *p_root  = NULL;
   cJSON *p_rooms = NULL;
   cJSON *p_room  = NULL;
   char  *p_post  = NULL;
   bool   ok      = false;
   int    i       = 0;

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
   (void)cJSON_AddNumberToObject(p_root, "batt_pir",     g_state.pir_batt);
   (void)cJSON_AddNumberToObject(p_root, "batt_dr1",     g_state.dr1_batt);
   (void)cJSON_AddNumberToObject(p_root, "batt_dr2",     g_state.dr2_batt);
   (void)cJSON_AddNumberToObject(p_root, "batt_lck",     g_state.lock_batt);
   (void)cJSON_AddNumberToObject(p_root, "motor_online", g_state.motor_online);

   p_rooms = cJSON_CreateArray();
   if (NULL != p_rooms)
   {
      for (i = 0; i < ROOM_COUNT; i++)
      {
         p_room = cJSON_CreateObject();
         if (NULL != p_room)
         {
            (void)cJSON_AddNumberToObject(p_room, "sensor_id", rooms[i].sensor_id);
            (void)cJSON_AddStringToObject(p_room, "room",      rooms[i].room);
            (void)cJSON_AddStringToObject(p_room, "state",     rooms[i].state);
            (void)cJSON_AddStringToObject(p_room, "location",  rooms[i].location);
            (void)cJSON_AddItemToArray(p_rooms, p_room);
         }
      }
      (void)cJSON_AddItemToObject(p_root, "rooms", p_rooms);
   }

   p_post = cJSON_PrintUnformatted(p_root);
   cJSON_Delete(p_root);

   if (NULL == p_post) { ESP_LOGE(TAG, "cJSON serialize failed"); return; }

   ESP_LOGI(TAG, "AWS send: motion=%lu temp=%d",
            g_state.motion_count, g_state.avg_temp);

   ok = send_to_aws(p_post);

   if (ok)
   {
      parse_control_response();
      ESP_LOGI(TAG, "AWS sent successfully");
   }
   else
   {
      ESP_LOGE(TAG, "AWS send failed");
      trinity_log_event("EVENT: AWS_SEND_FAIL\n");
   }

   cJSON_free(p_post);
}

void aws_manager_init(void)
{
   ESP_LOGI(TAG, "AWS manager initialized");
}

void aws_manager_task(EventGroupHandle_t p_wifi_eg,
                      EventGroupHandle_t p_events)
{
   EventBits_t bits      = 0;
   TickType_t  last_send = 0;
   TickType_t  now       = 0;

   (void)xEventGroupWaitBits(p_wifi_eg, WIFI_CONNECTED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);

   ESP_LOGI(TAG, "AWS task running");

   last_send = xTaskGetTickCount();

   while (1)
   {
      /* ---- Trinity: kick WDT every DRAIN_INTERVAL_MS (2 s) ---- */
      trinity_wdt_kick();

      bits = xEventGroupWaitBits(p_events,
                                  EVT_ALL_MASK,
                                  pdTRUE,
                                  pdFALSE,
                                  pdMS_TO_TICKS(DRAIN_INTERVAL_MS));

      drain_queues(bits);

      now = xTaskGetTickCount();
      if ((now - last_send) < pdMS_TO_TICKS(AWS_SEND_INTERVAL_MS))
      {
         continue;
      }

      last_send = now;
      send_state_to_aws();
   }
}
