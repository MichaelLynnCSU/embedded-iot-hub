/******************************************************************************
 * \file    motor_control.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Motor control task and JSON parser for ESP32-C3 motor node.
 *
 * \details Pure PWM receiver. Hub sends {"pwm": X} over TCP and this node
 *          applies the duty directly via LEDC. All temperature sensing,
 *          PID control, and duty calculation run on the ESP32 hub.
 *          Battery SOC is read and sent back during tcp_client_exchange().
 *
 * \note    Deep sleep idle (2026-05-XX):
 *          tcp_server.c removed -- motor is now the TCP client. Connection is
 *          ephemeral: wake, connect, receive PWM, send batt, sleep.
 *          Idle vTaskDelay() loop replaced with esp_deep_sleep_start() so
 *          the radio and CPU power rail are fully gated between wakes.
 *          MOTOR_DEEP_SLEEP_US defined in main.h (default 30 s).
 *
 *          g_client_sock / g_client_sock_valid removed -- battery reporting
 *          now happens inside tcp_client_exchange() immediately after the PWM
 *          frame is received, before the socket closes.
 *
 *          get_pwm_duty() accessor added so tcp_client.c can read the duty
 *          that parse_tcp_json() last wrote without accessing g_pwm_duty
 *          directly.
 *
 * \note    Power saving (2026-05-04, carried forward):
 *          Knob ADC and temp/PID logic already removed. Hub owns all control
 *          decisions. Node applies whatever duty it receives.
 *
 * \note    WDT fix (2026-05-04, carried forward):
 *          WiFi wait uses 1 s kicked loop. Deep sleep replaces both the old
 *          vTaskDelay() idle path and the broken esp_light_sleep_start()
 *          path -- no WIFI_PS_NONE conflict because the radio is fully off
 *          during deep sleep.
 *
 * \note    Battery reporting (2026-04-27, carried forward):
 *          Sends SOC percent {"batt_motor": 87}, not raw mV.
 *          mv_to_soc() defined in battery.h.
 ******************************************************************************/

#include "motor_control.h"
#include "tcp_client.h"
#include "wifi.h"
#include "battery.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "cJSON.h"
#include "trinity_log.h"

static const char *TAG = "MOTOR_CTRL";

#define MOTOR_PWM_PIN        GPIO_NUM_2
#define MOTOR_IN1_PIN        GPIO_NUM_3
#define MOTOR_IN2_PIN        GPIO_NUM_4

#define PWM_TIMER            LEDC_TIMER_0
#define PWM_MODE             LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL          LEDC_CHANNEL_0
#define PWM_DUTY_RES_LEDC    LEDC_TIMER_13_BIT
#define PWM_FREQUENCY        2000

static volatile uint32_t g_pwm_duty         = 0;
static int               g_batt_last_good_mv = 0;

#define LOOP_SLICE_MS   1000u

/*----------------------------------------------------------------------------*/

int motor_init(void)
{
    esp_err_t err = ESP_OK;

    err  = gpio_set_direction(MOTOR_IN1_PIN, GPIO_MODE_OUTPUT);
    err |= gpio_set_direction(MOTOR_IN2_PIN, GPIO_MODE_OUTPUT);
    err |= gpio_set_level(MOTOR_IN1_PIN, 0);
    err |= gpio_set_level(MOTOR_IN2_PIN, 0);

    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Motor GPIO init failed (%d)", err);
        return -1;
    }

    ESP_LOGI(TAG, "Motor GPIO init OK (IN1=GPIO%d, IN2=GPIO%d)",
             MOTOR_IN1_PIN, MOTOR_IN2_PIN);
    return 0;
}

/*----------------------------------------------------------------------------*/

void motor_enable(void)
{
    ESP_LOGI(TAG, "[MOTOR] motor_enable()");
    (void)gpio_set_level(MOTOR_IN1_PIN, 1);
    (void)gpio_set_level(MOTOR_IN2_PIN, 0);
}

/*----------------------------------------------------------------------------*/

uint32_t get_pwm_duty(void)
{
    return g_pwm_duty;
}

/*----------------------------------------------------------------------------*/

void parse_tcp_json(const char *p_buf)
{
    cJSON   *p_json = NULL;
    cJSON   *p_pwm  = NULL;
    uint32_t duty   = 0;

    p_json = cJSON_Parse(p_buf);
    if (NULL == p_json) { return; }

    p_pwm = cJSON_GetObjectItem(p_json, "pwm");
    if (NULL != p_pwm)
    {
        duty = (uint32_t)p_pwm->valueint;
        if (duty > PWM_DUTY_MAX) { duty = PWM_DUTY_MAX; }
        g_pwm_duty = duty;
        ESP_LOGI(TAG, "[PWM] duty=%u", duty);
        trinity_log_event("EVENT: PWM_UPDATE\n");
    }

    cJSON_Delete(p_json);
}

/*----------------------------------------------------------------------------*/

void motor_task(void *p_arg)
{
    uint32_t last_stats_ms  = 0u;
    uint32_t now_ms         = 0u;
    uint32_t duty           = 0u;
    int      batt_mv        = -1;

    (void)p_arg;

    trinity_wdt_add();

    ESP_LOGI(TAG, "[MOTOR_TASK] started, waiting for WiFi");

    while (!(xEventGroupWaitBits(g_wifi_eg, WIFI_CONNECTED_BIT,
                                  pdFALSE, pdTRUE,
                                  pdMS_TO_TICKS(1000)) & WIFI_CONNECTED_BIT))
    {
        trinity_wdt_kick();
    }

    ESP_LOGI(TAG, "[MOTOR_TASK] WiFi up, connecting to hub");

    last_stats_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    /* ------------------------------------------------------------------ *
     * Single wake-cycle: connect to hub, receive PWM, send batt, apply.  *
     * ------------------------------------------------------------------ */
    duty = tcp_client_exchange();

    (void)ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    (void)ledc_update_duty(PWM_MODE, PWM_CHANNEL);

    /* Battery sag filter -- log only; reporting already done in exchange */
    batt_mv = battery_read_mv();
    if (batt_mv > 0)
    {
        if ((g_batt_last_good_mv > 0) &&
            (batt_mv < (g_batt_last_good_mv - BATT_SAG_REJECT_MV)))
        {
            ESP_LOGW(TAG, "[BATT] Rejected sag reading %d mV (last good %d mV)",
                     batt_mv, g_batt_last_good_mv);
        }
        else
        {
            g_batt_last_good_mv = batt_mv;
        }
    }

    now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now_ms - last_stats_ms) >= STATS_INTERVAL_MS)
    {
        trinity_log_heap_stats();
        trinity_log_task_stats();
    }

    if (0u == duty)
    {
        /* PWM=0: motor idle -- deep sleep until next scheduled wake.
         *
         * Deep sleep gates the CPU and radio power rail entirely.
         * No WIFI_PS_NONE conflict (radio is fully off).
         * RTC timer wakes the chip after MOTOR_DEEP_SLEEP_US microseconds. */
        ESP_LOGI(TAG, "[MOTOR_TASK] idle -- entering deep sleep for %llu us",
                 (unsigned long long)MOTOR_DEEP_SLEEP_US);
        trinity_log_event("EVENT: MOTOR_DEEP_SLEEP\n");
        esp_sleep_enable_timer_wakeup(MOTOR_DEEP_SLEEP_US);
        esp_deep_sleep_start();
        /* unreachable -- deep sleep restarts from app_main */
    }
    else
    {
        /* PWM>0: motor running -- short delay then check hub again.
         * vTaskDelay() keeps WiFi driver alive while duty is applied. */
        ESP_LOGI(TAG, "[MOTOR_TASK] active (duty=%lu) -- delay %ums",
                 (unsigned long)duty, (unsigned)MOTOR_LOOP_MS);

        uint32_t elapsed = 0u;
        while (elapsed < MOTOR_LOOP_MS)
        {
            trinity_wdt_kick();
            vTaskDelay(pdMS_TO_TICKS(LOOP_SLICE_MS));
            elapsed += LOOP_SLICE_MS;
        }

        /* Re-connect hub for updated duty on next iteration.
         * Restart task loop by returning to app_main via deep sleep so
         * stack and WiFi state are always fresh. */
        ESP_LOGI(TAG, "[MOTOR_TASK] active cycle done -- deep sleep for %llu us",
                 (unsigned long long)MOTOR_DEEP_SLEEP_US);
        trinity_log_event("EVENT: MOTOR_CYCLE_DONE\n");
        esp_sleep_enable_timer_wakeup(MOTOR_DEEP_SLEEP_US);
        esp_deep_sleep_start();
        /* unreachable */
    }
}

/*----------------------------------------------------------------------------*/

void pwm_init(void)
{
    ledc_timer_config_t timer =
    {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_DUTY_RES_LEDC,
        .freq_hz         = PWM_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t channel =
    {
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .gpio_num   = MOTOR_PWM_PIN,
        .duty       = 0,
    };
    (void)ledc_timer_config(&timer);
    (void)ledc_channel_config(&channel);
    ESP_LOGI(TAG, "[PWM] init OK (GPIO%d, %dHz, 13-bit)",
             MOTOR_PWM_PIN, PWM_FREQUENCY);
}
