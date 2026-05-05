/******************************************************************************
 * \file    main.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Entry point for ESP32-C3 motor controller node.
 *
 * \details Initialises hardware, WiFi, and spawns two tasks:
 *          - tcp_rx_task:  accepts hub connection, dispatches {"pwm": X}
 *          - motor_task:   applies PWM duty, reports battery SOC
 *
 * \note    Power saving (2026-05-04):
 *          Knob ADC removed -- motor is now a pure PWM receiver. Hub owns
 *          all temperature sensing and PID control. adc_init() removed from
 *          startup. battery_init() retains its own ADC handle internally.
 *          motor_task enters light sleep when PWM=0 and no client connected.
 ******************************************************************************/

#include "wifi.h"
#include "tcp_server.h"
#include "motor_control.h"
#include "battery.h"
#include "main.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "trinity_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TCP_TASK_STACK   6144
#define MOTOR_TASK_STACK 4096
#define TASK_PRIORITY    5

static const char *TAG = "APP_MAIN";

void app_main(void)
{
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

    if (0 != battery_init())
    {
        ESP_LOGW(TAG, "Battery init failed -- batt_motor will not report");
    }

    trinity_log_event("EVENT: HARDWARE_READY\n");

    xTaskCreate(tcp_rx_task, "tcp_rx", TCP_TASK_STACK,  NULL, TASK_PRIORITY, NULL);
    xTaskCreate(motor_task,  "motor",  MOTOR_TASK_STACK, NULL, TASK_PRIORITY, NULL);
}
