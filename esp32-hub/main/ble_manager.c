/******************************************************************************
 * \file ble_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief BLE manager for ESP32 hub node.
 *
 * \details Owns the BLE stack lifecycle, connection scheduler, and
 *          per-device age tracking. Initializes the BLE stack, creates
 *          the connection scheduler task, and starts scanning.
 *
 *          Initialization order:
 *          1. ble_stack_init()   — BT controller + Bluedroid
 *          2. connect_queue      — connection scheduler queue
 *          3. scheduler_task     — connection serialiser task
 *          4. ble_scan_preinit() — scan module pre-init
 *          5. ble_gattc_init()   — GATT client registration
 *          6. ble_scan_start()   — begin scanning
 *
 *          Connection scheduler:
 *          - connect_queue serialises all GATT open requests
 *          - g_connection_in_progress is a radio-level semaphore
 *          - Watchdog clears stalled connections after CONNECT_WATCHDOG_MS
 *
 * \note    WDT fix (2026-03-21):
 *          Idle loop vTaskDelay reduced from 10000 ms to 2000 ms and
 *          trinity_wdt_kick() added at the top of the idle loop.
 *          The 10 s delay was 2× the WDT timeout, guaranteeing a trigger
 *          even on a healthy boot.
 *          trinity_log_event() renamed to trinity_log_event() throughout.
 ******************************************************************************/

#include "config.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "ble_manager.h"
#include "ble_internal.h"
#include "trinity_log.h"
#include <string.h>

#define SCHED_POLL_MS        100   /**< scheduler queue poll interval ms          */
#define SCHED_TASK_STACK     2048  /**< scheduler task stack size bytes            */
#define SCHED_TASK_PRIORITY  5     /**< scheduler task priority                    */
#define BT_STACK_DELAY_MS    200   /**< delay between BT stack init steps ms       */
#define BLE_IDLE_DELAY_MS    2000  /**< idle loop delay ms (must be < WDT timeout) */

static const char *TAG = "BLE_MGR"; /**< ESP log tag */

static uint32_t g_dev_last_seen_ms[DEV_IDX_COUNT] = {0};
static uint32_t g_connect_start_ms = 0;

QueueHandle_t  connect_queue            = NULL;
volatile bool  g_connection_in_progress = false;

void stamp_device(BLE_DEV_IDX_E idx)
{
   if (idx < DEV_IDX_COUNT)
   {
      g_dev_last_seen_ms[idx] = xTaskGetTickCount() * portTICK_PERIOD_MS;
   }
}

uint16_t ble_get_device_age_s(int idx)
{
   uint32_t now = 0;

   if ((0 > idx) || (idx >= DEV_IDX_COUNT)) { return 0xFFFF; }
   if (0 == g_dev_last_seen_ms[idx])        { return 0xFFFF; }

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;
   return (uint16_t)((now - g_dev_last_seen_ms[idx]) / 1000);
}

void ble_scheduler_notify_done(void)
{
   g_connection_in_progress = false;
   g_connect_start_ms       = 0;
}

static void scheduler_task(void *p_arg)
{
   BLE_CONNECT_REQUEST_T req;
   uint32_t now = 0;

   (void)p_arg;

   while (1)
   {
      if (g_connection_in_progress && (0 != g_connect_start_ms))
      {
         now = xTaskGetTickCount() * portTICK_PERIOD_MS;

         if ((now - g_connect_start_ms) > CONNECT_WATCHDOG_MS)
         {
            ESP_LOGW(TAG, "[SCHED] Watchdog: connection stalled, resetting");
            trinity_log_event("EVENT: BLE_CONNECT_WATCHDOG\n");
            g_connection_in_progress = false;
            g_connect_start_ms       = 0;
         }
      }

      if (g_connection_in_progress)
      {
         vTaskDelay(pdMS_TO_TICKS(SCHED_POLL_MS));
         continue;
      }

      if (pdTRUE == xQueueReceive(connect_queue,
                                   &req,
                                   pdMS_TO_TICKS(SCHED_POLL_MS)))
      {
         if (NULL != req.connect_fn)
         {
            ESP_LOGI(TAG, "[SCHED] Issuing connect");
            g_connection_in_progress = true;
            g_connect_start_ms       = xTaskGetTickCount() * portTICK_PERIOD_MS;
            req.connect_fn();
         }
      }
   }
}

static void ble_stack_init(void)
{
   esp_err_t ret = ESP_OK;
   esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

   ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "BT mem release failed: %s", esp_err_to_name(ret));
   }

   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(BT_STACK_DELAY_MS));

   ret = esp_bt_controller_init(&bt_cfg);
   if (ESP_OK != ret)
   {
      trinity_log_event("EVENT: BLE_CONTROLLER_INIT_FAIL\n");
      ESP_LOGE(TAG, "Controller init failed");
      return;
   }

   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(BT_STACK_DELAY_MS));

   ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
   if (ESP_OK != ret) { ESP_LOGE(TAG, "Controller enable failed"); return; }

   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(BT_STACK_DELAY_MS));

   ret = esp_bluedroid_init();
   if (ESP_OK != ret)
   {
      trinity_log_event("EVENT: BLE_BLUEDROID_INIT_FAIL\n");
      ESP_LOGE(TAG, "Bluedroid init failed");
      return;
   }

   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(BT_STACK_DELAY_MS));

   ret = esp_bluedroid_enable();
   if (ESP_OK != ret) { ESP_LOGE(TAG, "Bluedroid enable failed"); return; }

   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(BT_STACK_DELAY_MS));

   ESP_LOGI(TAG, "BLE stack initialised");
}

void ble_manager_init(void)
{
   ESP_LOGI(TAG, "BLE manager init");
}

void ble_manager_task(EventGroupHandle_t p_system_eg,
                      EventGroupHandle_t p_ble_eg)
{
   (void)p_system_eg;

   ESP_LOGI(TAG, "BLE task running on core %d", xPortGetCoreID());

   /* ---- Trinity: kick before the 4 s pre-init delay so the WDT does
    *      not fire while waiting for other subsystems to settle.    ---- */
   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(BLE_INIT_DELAY_MS));

   ble_stack_init();

   connect_queue = xQueueCreate(CONNECT_QUEUE_DEPTH, sizeof(BLE_CONNECT_REQUEST_T));
   configASSERT(connect_queue);

   xTaskCreate(scheduler_task, "ble_sched", SCHED_TASK_STACK,
               NULL, SCHED_TASK_PRIORITY, NULL);

   ble_scan_preinit();
   ble_gattc_init();
   vTaskDelay(pdMS_TO_TICKS(BT_STACK_DELAY_MS));

   ble_scan_start();

   (void)xEventGroupSetBits(p_ble_eg, BLE_INITIALIZED_BIT);
   ESP_LOGI(TAG, "BLE initialised — scan running");

   while (1)
   {
      /* ---- Trinity: kick WDT every BLE_IDLE_DELAY_MS (2 s) ---- */
      trinity_wdt_kick();
      ESP_LOGI(TAG, "[BLE] PIR=%s LIGHT=%s LOCK=%s REEDS=%d",
         (ble_get_device_age_s(BLE_DEV_PIR)   < 30) ? "online" : "offline",
         (ble_get_device_age_s(BLE_DEV_LIGHT)  < 30) ? "online" : "offline",
         (ble_get_device_age_s(BLE_DEV_LOCK)   < 30) ? "online" : "offline",
          ble_get_reed_count());
      vTaskDelay(pdMS_TO_TICKS(BLE_IDLE_DELAY_MS));
   }
}

int ble_get_motion_count(void)
{
   extern int motion_count;
   return motion_count;
}

uint8_t ble_get_light_state(void)
{
   return (uint8_t)ble_light_get_state();
}

uint8_t ble_get_lock_state(void)
{
   return (uint8_t)ble_lock_get_state();
}

int ble_get_pir_batt(void)
{
   extern int pir_batt;
   return pir_batt;
}

int ble_get_pir_occupied(void)
{
   extern int g_pir_occupied;
   return g_pir_occupied;
}

int ble_get_dr1_batt(void)
{
   extern int dr1_batt;
   return dr1_batt;
}

int ble_get_dr2_batt(void)
{
   extern int dr2_batt;
   return dr2_batt;
}

int ble_get_lock_batt(void)
{
   return ble_lock_get_batt();
}

void ble_update_room_sensor(int sensor_id, const char *p_state)
{
   int i = 0;

   for (i = 0; i < ROOM_COUNT; i++)
   {
      if (sensor_id == rooms[i].sensor_id)
      {
         rooms[i].state = p_state;
         break;
      }
   }
}
