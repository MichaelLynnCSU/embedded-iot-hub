/******************************************************************************
 * \file main.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Application entry point and FreeRTOS task definitions for
 *        ESP32 hub node (Vroom).
 *
 * \details Trinity additions:
 *          - trinity_wdt_init()  called in app_main after trinity_log_init()
 *          - trinity_wdt_add()   called in each task AFTER WiFi is confirmed
 *            up (deferred from task start to avoid WDT starvation during the
 *            ~4-5 s WiFi association window)
 *          - trinity_wdt_kick()  called in each task's main loop
 *          - trinity_log_heap_stats() / trinity_log_task_stats() called
 *            periodically from tcp_task (every ~60s)
 *
 * \note  WDT fix (2026-03-21):
 *        Previously every task called trinity_wdt_add() at startup then
 *        immediately blocked inside its manager on WIFI_CONNECTED_BIT.
 *        Because all tasks were registered but none ever ran, the WDT
 *        fired after exactly 5 s on every boot where WiFi association
 *        took longer than the timeout.  The fix is to wait for WiFi in a
 *        kicked polling loop BEFORE registering with the WDT, so the WDT
 *        only tracks tasks that are actually executing.
 ******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "config.h"
#include "ble_manager.h"
#include "wifi_manager.h"
#include "tcp_manager.h"
#include "aws_manager.h"
#include "uart_manager.h"
#include "vroom_bus.h"
#include "trinity_log.h"

#define UART_BAUD_RATE          115200   /**< STM32 UART baud rate             */
#define STATS_INTERVAL_MS       60000u   /**< heap/task stats interval          */
#define WIFI_POLL_INTERVAL_MS   2000u    /**< WiFi-wait polling / WDT kick rate */

static const char *TAG = "VROOM";

static EventGroupHandle_t g_system_eg;
static EventGroupHandle_t g_wifi_eg;
static EventGroupHandle_t g_ble_eg;

static TaskHandle_t g_wifi_task_handle;
static TaskHandle_t g_ble_task_handle;
static TaskHandle_t g_uart_task_handle;
static TaskHandle_t g_tcp_task_handle;
static TaskHandle_t g_aws_task_handle;

ROOM_SENSOR_T rooms[] =
{
   {1, "bedroom", "open",   "main_door"},
   {7, "bedroom", "closed", "closet_door"},
};

typedef struct { EventGroupHandle_t bus_events; } TCP_TASK_PARAMS_T;
typedef struct { EventGroupHandle_t bus_events; } AWS_TASK_PARAMS_T;

static TCP_TASK_PARAMS_T g_tcp_params;
static AWS_TASK_PARAMS_T g_aws_params;

static void wifi_task(void *p_arg);
static void ble_task(void *p_arg);
static void uart_task(void *p_arg);
static void tcp_task(void *p_arg);
static void aws_task(void *p_arg);

/******************************************************************************
 * \brief Block until WiFi is connected, kicking the WDT each poll interval.
 *
 * \details Called by every task BEFORE trinity_wdt_add() so no task is
 *          registered with the WDT while it is parked waiting for the
 *          network.  Once this returns, the caller immediately calls
 *          trinity_wdt_add() and enters its live loop.
 ******************************************************************************/
static void wait_for_wifi_kicked(void)
{
   while (!(xEventGroupGetBits(g_wifi_eg) & WIFI_CONNECTED_BIT))
   {
      vTaskDelay(pdMS_TO_TICKS(WIFI_POLL_INTERVAL_MS));
   }
}

static void uart_hw_init(void)
{
   uart_config_t uart_config =
   {
      .baud_rate  = UART_BAUD_RATE,
      .data_bits  = UART_DATA_8_BITS,
      .parity     = UART_PARITY_DISABLE,
      .stop_bits  = UART_STOP_BITS_1,
      .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
   };

   ESP_ERROR_CHECK(uart_param_config(UART_STM32, &uart_config));
   ESP_ERROR_CHECK(uart_set_pin(UART_STM32,
                                 STM32_TX_PIN, STM32_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
   ESP_ERROR_CHECK(uart_driver_install(UART_STM32,
                                        UART_BUF_SIZE, UART_BUF_SIZE,
                                        0, NULL, 0));
   ESP_LOGI(TAG, "UART initialized");
}

static void nvs_hw_init(void)
{
   esp_err_t ret = nvs_flash_init();

   if ((ESP_ERR_NVS_NO_FREE_PAGES == ret) ||
       (ESP_ERR_NVS_NEW_VERSION_FOUND == ret))
   {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
   }

   ESP_ERROR_CHECK(ret);
   ESP_LOGI(TAG, "NVS initialized");
}

static void create_tasks(void)
{
   ESP_LOGI(TAG, "Creating tasks...");

   if (pdPASS != xTaskCreatePinnedToCore(wifi_task, "WifiTask",
                                          STACK_SIZE_WIFI_INIT, NULL, 1,
                                          &g_wifi_task_handle, 0))
   { trinity_log_event("EVENT: TASK_FAIL | WIFI\n"); }

   vTaskDelay(pdMS_TO_TICKS(TASK_CREATION_DELAY_MS));

   if (pdPASS != xTaskCreatePinnedToCore(ble_task, "BleTask",
                                          STACK_SIZE_BLE_INIT, NULL, 1,
                                          &g_ble_task_handle, 1))
   { trinity_log_event("EVENT: TASK_FAIL | BLE\n"); }

   vTaskDelay(pdMS_TO_TICKS(TASK_CREATION_DELAY_MS));

   if (pdPASS != xTaskCreate(uart_task, "UartTask",
                               STACK_SIZE_UART_RX, NULL, 1,
                               &g_uart_task_handle))
   { trinity_log_event("EVENT: TASK_FAIL | UART\n"); }

   vTaskDelay(pdMS_TO_TICKS(TASK_CREATION_DELAY_MS));

   if (pdPASS != xTaskCreate(tcp_task, "TcpTask",
                               STACK_SIZE_TCP_SEND, &g_tcp_params, 1,
                               &g_tcp_task_handle))
   { trinity_log_event("EVENT: TASK_FAIL | TCP\n"); }

   vTaskDelay(pdMS_TO_TICKS(TASK_CREATION_DELAY_MS));

   if (pdPASS != xTaskCreate(aws_task, "AwsTask",
                               STACK_SIZE_AWS_SEND, &g_aws_params, 1,
                               &g_aws_task_handle))
   { trinity_log_event("EVENT: TASK_FAIL | AWS\n"); }

   ESP_LOGI(TAG, "All tasks created");
   ESP_LOGI(TAG, "Free heap after task creation: %lu bytes",
            esp_get_free_heap_size());
}

void app_main(void)
{
   ESP_LOGI(TAG, "Starting VROOM initialization...");

   nvs_flash_init();
   trinity_log_dump_previous();
   trinity_log_init();

   /* ---- Trinity: arm task watchdog ---- */
   trinity_wdt_init();

   nvs_hw_init();

   g_system_eg = xEventGroupCreate();
   g_wifi_eg   = xEventGroupCreate();
   g_ble_eg    = xEventGroupCreate();

   bus_init();

   g_tcp_params.bus_events = bus_register_subscriber();
   g_aws_params.bus_events = bus_register_subscriber();

   if ((NULL == g_tcp_params.bus_events) || (NULL == g_aws_params.bus_events))
   {
      ESP_LOGE(TAG, "Bus subscriber registration failed");
   }

   uart_hw_init();
   wifi_manager_init();
   ble_manager_init();
   uart_manager_init();
   tcp_manager_init();

   create_tasks();

   (void)xEventGroupSetBits(g_system_eg, ALL_TASKS_CREATED_BIT);
   ESP_LOGI(TAG, "System ready - tasks starting");
}

/******************************************************************************
 * \brief WiFi manager task.
 *
 * \details wifi_manager_task() drives association internally, so it cannot
 *          wait for WIFI_CONNECTED_BIT before starting.  It registers with
 *          the WDT immediately and must kick from within its own loop.
 *          All other tasks defer registration until wifi_task sets
 *          WIFI_CONNECTED_BIT, so only this task is tracked during the
 *          association window.
 ******************************************************************************/
static void wifi_task(void *p_arg)
{
   (void)p_arg;

   (void)xEventGroupWaitBits(g_system_eg, ALL_TASKS_CREATED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);

   /* ---- Trinity: register NOW — wifi_manager_task() must kick internally ---- */
   trinity_wdt_add();

   wifi_manager_task(g_system_eg, g_wifi_eg);
   vTaskDelete(NULL);
}

static void ble_task(void *p_arg)
{
   (void)p_arg;

   (void)xEventGroupWaitBits(g_system_eg, ALL_TASKS_CREATED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);

   /* ---- Trinity: wait for WiFi before registering — avoids WDT starvation
    *      during the association window.                                 ---- */
   wait_for_wifi_kicked();
   trinity_wdt_add();

   ble_manager_task(g_system_eg, g_ble_eg);
   vTaskDelete(NULL);
}

static void uart_task(void *p_arg)
{
   (void)p_arg;

   (void)xEventGroupWaitBits(g_system_eg, ALL_TASKS_CREATED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);

   /* ---- Trinity: defer WDT registration until WiFi is up ---- */
   wait_for_wifi_kicked();
   trinity_wdt_add();

   uart_manager_task();
   vTaskDelete(NULL);
}

/******************************************************************************
 * \brief TCP manager task — also owns periodic heap/task stats logging.
 ******************************************************************************/
static void tcp_task(void *p_arg)
{
   TCP_TASK_PARAMS_T *p_params = (TCP_TASK_PARAMS_T *)p_arg;

   (void)xEventGroupWaitBits(g_system_eg, ALL_TASKS_CREATED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);

   /* ---- Trinity: defer WDT registration until WiFi is up ---- */
   wait_for_wifi_kicked();
   trinity_wdt_add();

   tcp_manager_task(g_system_eg, g_wifi_eg, p_params->bus_events);
   vTaskDelete(NULL);
}

static void aws_task(void *p_arg)
{
   AWS_TASK_PARAMS_T *p_params = (AWS_TASK_PARAMS_T *)p_arg;

   (void)xEventGroupWaitBits(g_system_eg, ALL_TASKS_CREATED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);

   /* ---- Trinity: defer WDT registration until WiFi is up ---- */
   wait_for_wifi_kicked();
   trinity_wdt_add();

   aws_manager_task(g_wifi_eg, p_params->bus_events);
   vTaskDelete(NULL);
}
