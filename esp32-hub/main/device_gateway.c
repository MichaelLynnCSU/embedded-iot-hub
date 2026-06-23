/******************************************************************************
 * \file device_gateway.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief AWS IoT Lambda manager for ESP32 hub node.
 *
 * \details Maintains a local snapshot of all BLE and UART sensor state
 *          by draining the vroom bus queues on every wakeup. Sends a
 *          consolidated JSON payload to AWS Lambda every 5 minutes.
 *          Parses the Lambda response for PI controller parameters and
 *          light control updates.
 *
 *          Send interval: AWS_SEND_INTERVAL_MS (5 minutes)
 *          Drain interval: 2 seconds (keeps state fresh between sends,
 *          and doubles as the WDT kick interval)
 *
 *          JSON payload fields:
 *          avg_temp, motion_count, light_state, lock_state,
 *          batt_pir, batt_dr1, batt_dr2, batt_lck, motor_online,
 *          temps[], rooms[]
 *
 *          Lambda response fields (PI controller):
 *          kp, ki, kd, setpoint
 *          Consumed by tcp_manager.c via gateway_get_kp/ki/kd/setpoint().
 *
 * \note    WDT fix (2026-03-21): (unchanged)
 * \note    PI controller params (2026-05-04): (unchanged)
 * \note    TempSensor BLE (2026-06-02):
 *          BLE_TEMP_PAYLOAD_T drained from mb_ble_temp. Per-slot temp and
 *          batt tracked in g_state.temp_slots[]. "temps" array added to
 *          Lambda payload. "temp" field is whole degrees C matching
 *          avg_temp convention — divide by 10 at serialization only.
 * \note    AWS decoupled (2026-06-23):
 *          Direct Lambda send removed pending BeagleBone /ingest endpoint.
 *          Task loop and queue drain preserved. PI controller continues on
 *          compile-time defaults from config.h. Re-enable with ENABLE_AWS.
 ******************************************************************************/

#include "config.h"
#include "network_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include "ble_manager.h"
#include "ble_temp.h"
#include "uart_manager.h"
#include "device_gateway.h"
#include "vroom_bus.h"
#include "trinity_log.h"

#ifdef ENABLE_AWS
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#endif

#define DRAIN_INTERVAL_MS   2000   /**< queue drain interval ms */
#define HTTP_TIMEOUT_MS     10000  /**< HTTP request timeout ms */
#define HTTP_STATUS_OK      200    /**< expected HTTP success status */

static const char *TAG = "DEV_GW"; /**< ESP log tag */

/* PI controller parameters — updated by Lambda response, read by tcp_manager */
static float g_aws_kp       = DEFAULT_AWS_KP;
static float g_aws_ki       = DEFAULT_AWS_KI;
static float g_aws_kd       = DEFAULT_AWS_KD;
static int   g_aws_setpoint = DEFAULT_AWS_SETPOINT;

typedef struct
{
   int16_t  temp_decidegc; /*!< temperature in tenths of °C  */
   int      batt;          /*!< battery SOC percent          */
   bool     active;        /*!< slot has been seen           */
} AWS_TEMP_SLOT_T;

typedef struct
{
   int              avg_temp;
   uint32_t         motion_count;
   int              pir_batt;
   uint8_t          light_state;
   uint8_t          lock_state;
   int              lock_batt;
   int              dr1_batt;
   int              dr2_batt;
   uint8_t          motor_online;
   AWS_TEMP_SLOT_T  temp_slots[MAX_TEMPS];
   int              temp_count;
} AWS_STATE_T;

static AWS_STATE_T g_state =
{
   .avg_temp   = DEFAULT_AVG_TEMP,
   .pir_batt   = -1,
   .lock_batt  = -1,
   .dr1_batt   = -1,
   .dr2_batt   = -1,
   .temp_count = 0,
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
   int                t_slot = 0;

   /* ---- PIR ---- */
   if (pdTRUE == xQueueReceive(sub.mb_pir, &p_pir, 0))
   {
      g_state.motion_count = p_pir.count;
      g_state.pir_batt     = p_pir.batt;
   }

   /* ---- REED ---- */
   if (pdTRUE == xQueueReceive(sub.mb_reed, &p_reed, 0))
   {
      if (1 == p_reed.id) { g_state.dr1_batt = p_reed.batt; }
      else                { g_state.dr2_batt = p_reed.batt; }
   }

   /* ---- LOCK ---- */
   if (pdTRUE == xQueueReceive(sub.mb_lock, &p_lock, 0))
   {
      g_state.lock_state = p_lock.state;
      g_state.lock_batt  = p_lock.batt;
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

   /* ---- MOTOR ---- */
   if (pdTRUE == xQueueReceive(sub.mb_motor, &p_motor, 0))
   {
      g_state.motor_online = p_motor.online;
   }

   /* ---- BLE TEMP ---- */
   if (pdTRUE == xQueueReceive(sub.mb_ble_temp, &p_ble_temp, 0))
   {
      t_slot = (int)p_ble_temp.id - 1;
      if ((0 <= t_slot) && (t_slot < MAX_TEMPS))
      {
         g_state.temp_slots[t_slot].temp_decidegc = p_ble_temp.temp_decidegc;
         g_state.temp_slots[t_slot].batt          = p_ble_temp.batt;
         g_state.temp_slots[t_slot].active        = true;
      }
   }

   /* refresh temp_count from slot table */
   g_state.temp_count = ble_get_temp_count();
}

/*----------------------------------------------------------------------------*/

#ifdef ENABLE_AWS

static char g_response_buffer[HTTP_RESPONSE_BUFFER_SIZE];
static int  g_response_len = 0;

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
   esp_err_t err     = ESP_OK;
   int       status  = 0;
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
   cJSON *p_json     = NULL;
   cJSON *p_kp       = NULL;
   cJSON *p_ki       = NULL;
   cJSON *p_kd       = NULL;
   cJSON *p_setpoint = NULL;
   cJSON *p_light    = NULL;

   if (0 == g_response_len) { return; }

   p_json = cJSON_Parse(g_response_buffer);
   if (NULL == p_json) { return; }

   p_kp       = cJSON_GetObjectItem(p_json, "kp");
   p_ki       = cJSON_GetObjectItem(p_json, "ki");
   p_kd       = cJSON_GetObjectItem(p_json, "kd");
   p_setpoint = cJSON_GetObjectItem(p_json, "setpoint");
   p_light    = cJSON_GetObjectItem(p_json, "light");

   if (NULL != p_kp)       { g_aws_kp       = (float)p_kp->valuedouble;  }
   if (NULL != p_ki)       { g_aws_ki       = (float)p_ki->valuedouble;  }
   if (NULL != p_kd)       { g_aws_kd       = (float)p_kd->valuedouble;  }
   if (NULL != p_setpoint) { g_aws_setpoint = p_setpoint->valueint;       }
   if (NULL != p_light)    { ble_send_light_command(p_light->valueint);   }

   ESP_LOGI(TAG, "Control updated: kp=%.3f ki=%.3f kd=%.3f setpoint=%d",
            g_aws_kp, g_aws_ki, g_aws_kd, g_aws_setpoint);

   cJSON_Delete(p_json);
}

static void send_state_to_aws(void)
{
   cJSON *p_root  = NULL;
   cJSON *p_rooms = NULL;
   cJSON *p_room  = NULL;
   cJSON *p_temps = NULL;
   cJSON *p_entry = NULL;
   char  *p_post  = NULL;
   bool   ok      = false;
   int    i       = 0;

   p_root = cJSON_CreateObject();
   if (NULL == p_root) { ESP_LOGE(TAG, "cJSON root alloc failed"); return; }

   (void)cJSON_AddNumberToObject(p_root, "avg_temp",     g_state.avg_temp);
   (void)cJSON_AddNumberToObject(p_root, "motion_count", g_state.motion_count);
   (void)cJSON_AddNumberToObject(p_root, "light_state",  g_state.light_state);
   (void)cJSON_AddNumberToObject(p_root, "lock_state",   g_state.lock_state);
   (void)cJSON_AddNumberToObject(p_root, "batt_pir",     g_state.pir_batt);
   (void)cJSON_AddNumberToObject(p_root, "batt_dr1",     g_state.dr1_batt);
   (void)cJSON_AddNumberToObject(p_root, "batt_dr2",     g_state.dr2_batt);
   (void)cJSON_AddNumberToObject(p_root, "batt_lck",     g_state.lock_batt);
   (void)cJSON_AddNumberToObject(p_root, "motor_online", g_state.motor_online);

   p_temps = cJSON_CreateArray();
   if (NULL != p_temps)
   {
      for (i = 0; (i < g_state.temp_count) && (i < MAX_TEMPS); i++)
      {
         if (!g_state.temp_slots[i].active) { continue; }
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddNumberToObject(p_entry, "id",   i + 1);
            (void)cJSON_AddNumberToObject(p_entry, "temp",
                                          g_state.temp_slots[i].temp_decidegc / 10);
            (void)cJSON_AddNumberToObject(p_entry, "batt",
                                          g_state.temp_slots[i].batt);
            (void)cJSON_AddItemToArray(p_temps, p_entry);
         }
      }
      (void)cJSON_AddItemToObject(p_root, "temps", p_temps);
   }
   (void)cJSON_AddNumberToObject(p_root, "temp_count", g_state.temp_count);

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

   ESP_LOGI(TAG, "AWS send: motion=%lu temp=%d temps=%d",
            g_state.motion_count, g_state.avg_temp, g_state.temp_count);

   ok = send_to_aws(p_post);

   if (ok) { parse_control_response(); ESP_LOGI(TAG, "AWS sent successfully"); }
   else    { ESP_LOGE(TAG, "AWS send failed"); trinity_log_event("EVENT: AWS_SEND_FAIL\n"); }

   cJSON_free(p_post);
}

#endif /* ENABLE_AWS */

/*******************************************************************************
 * Public getter functions
 ******************************************************************************/

float gateway_get_kp(void)       { return g_aws_kp;       }
float gateway_get_ki(void)       { return g_aws_ki;       }
float gateway_get_kd(void)       { return g_aws_kd;       }
int   gateway_get_setpoint(void) { return g_aws_setpoint; }

void device_gateway_init(void)
{
#ifdef ENABLE_AWS
   ESP_LOGI(TAG, "AWS manager initialized");
#else
   ESP_LOGI(TAG, "AWS manager stubbed — ENABLE_AWS not set");
#endif
}

void device_gateway_task(EventGroupHandle_t p_wifi_eg,
                      BUS_SUBSCRIBER_T   sub)
{
#ifdef ENABLE_AWS
   TickType_t last_send = 0;
   TickType_t now       = 0;
#endif

   (void)xEventGroupWaitBits(p_wifi_eg, WIFI_CONNECTED_BIT,
                              pdFALSE, pdTRUE, portMAX_DELAY);

   ESP_LOGI(TAG, "AWS task running (stub mode — drain only)");

#ifdef ENABLE_AWS
   last_send = xTaskGetTickCount();
#endif

   while (1)
   {
      trinity_wdt_kick();

      (void)xEventGroupWaitBits(sub.events, sub.mask, pdTRUE, pdFALSE,
                                pdMS_TO_TICKS(DRAIN_INTERVAL_MS));
      drain_queues(sub);

#ifdef ENABLE_AWS
      now = xTaskGetTickCount();
      if ((now - last_send) >= pdMS_TO_TICKS(AWS_SEND_INTERVAL_MS))
      {
         last_send = now;
         send_state_to_aws();
      }
#endif
   }
}
