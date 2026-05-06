/******************************************************************************
 * \file    main.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Entry point for ESP32-C3 motor controller node.
 *
 * \details Initialises hardware, WiFi, and spawns one task:
 *          - motor_task: connects to hub, applies PWM duty, deep sleeps.
 *
 *          tcp_rx_task removed -- motor is now the TCP client (tcp_client.c).
 *          There is no persistent listen socket. Each wake from deep sleep
 *          runs app_main fresh, calls tcp_client_exchange() once via
 *          motor_task, then returns to deep sleep.
 *
 * \note    Arch change (2026-05-XX):
 *          tcp_server.h replaced with tcp_client.h.
 *          TCP_TASK_STACK and tcp_rx_task spawn removed.
 *          MOTOR_TASK_STACK unchanged.
 *
 * \note    Power saving (2026-05-04, carried forward):
 *          Knob ADC removed. battery_init() retains its own ADC handle.
 *          motor_task enters deep sleep when exchange complete.
 ******************************************************************************/

#include "wifi.h"
#include "tcp_client.h"
#include "motor_control.h"
#include "battery.h"
#include "main.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "trinity_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_TASK_STACK 4096
#define TASK_PRIORITY    5

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "[MAIN] nvs_flash_init");
    (void)nvs_flash_init();

    ESP_LOGI(TAG, "[MAIN] trinity_log_dump_previous");
    trinity_log_dump_previous();

    ESP_LOGI(TAG, "[MAIN] trinity_log_init");
    trinity_log_init();

    ESP_LOGI(TAG, "ESP32-C3 Motor Controller Starting");

    ESP_LOGI(TAG, "[MAIN] trinity_wdt_init");
    trinity_wdt_init();

    ESP_LOGI(TAG, "[MAIN] wifi_init");
    wifi_init();
    ESP_LOGI(TAG, "[MAIN] wifi_init done");

    ESP_LOGI(TAG, "[MAIN] motor_init");
    if (0 != motor_init())
    {
        ESP_LOGE(TAG, "Motor GPIO init failed -- halting");
        return;
    }

    ESP_LOGI(TAG, "[MAIN] pwm_init");
    pwm_init();

    ESP_LOGI(TAG, "[MAIN] battery_init");
    if (0 != battery_init())
    {
        ESP_LOGW(TAG, "Battery init failed -- batt_motor will not report");
    }

    trinity_log_event("EVENT: HARDWARE_READY\n");
    ESP_LOGI(TAG, "[MAIN] HARDWARE_READY -- spawning motor_task");

    xTaskCreate(motor_task, "motor", MOTOR_TASK_STACK, NULL, TASK_PRIORITY, NULL);
    ESP_LOGI(TAG, "[MAIN] motor_task spawned");

    ESP_LOGI(TAG, "[MAIN] app_main returning");
}
