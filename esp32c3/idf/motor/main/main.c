/******************************************************************************
 * \file    main.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Entry point for ESP32-C3 motor controller node.
 ******************************************************************************/

#include "wifi.h"
#include "tcp_server.h"
#include "motor_control.h"
#include "battery.h"
#include "main.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "trinity_log.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TCP_TASK_STACK   6144
#define MOTOR_TASK_STACK 4096
#define TASK_PRIORITY    5

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    adc_oneshot_unit_handle_t adc_handle = NULL;

    (void)nvs_flash_init();
    trinity_log_dump_previous();
    trinity_log_init();

    ESP_LOGI(TAG, "ESP32-C3 Motor Controller Starting");

    trinity_wdt_init();

    wifi_init();

    if (0 != motor_init())
    {
        ESP_LOGE(TAG, "Motor GPIO init failed -- halting");
        return;
    }

    pwm_init();
    adc_init(&adc_handle);

    if (0 != battery_init(adc_handle))
    {
        ESP_LOGW(TAG, "Battery init failed -- batt_motor will not report");
    }

    trinity_log_event("EVENT: HARDWARE_READY\n");

    xTaskCreate(tcp_rx_task, "tcp_rx", TCP_TASK_STACK,  NULL, TASK_PRIORITY, NULL);
    xTaskCreate(motor_task,  "motor",  MOTOR_TASK_STACK, NULL, TASK_PRIORITY, NULL);
}
