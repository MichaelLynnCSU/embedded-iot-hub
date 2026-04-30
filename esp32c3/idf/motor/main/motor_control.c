/******************************************************************************
 * \file    motor_control.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Motor control task and JSON parser for ESP32-C3 motor node.
 *
 * \details PWM duty is derived from temperature (AWS mode) or knob ADC
 *          (manual mode). Battery is read every BATT_INTERVAL_MS and
 *          reported to the hub as SOC percent -- matching the nRF52840
 *          smart-lock convention so the hub treats both nodes identically.
 *
 * \note    Battery reporting (2026-04-27):
 *          Previously sent raw millivolts: {"batt_motor": 9145}
 *          Now sends SOC percent:          {"batt_motor": 87}
 *          mv_to_soc() defined in battery.h -- same pattern as smart-lock.
 *          Hub no longer needs to hardcode voltage thresholds or do math.
 *          BATT_JSON_BUF_SIZE in main.h covers {"batt_motor":100} + null.
 ******************************************************************************/

#include "motor_control.h"
#include "tcp_server.h"
#include "wifi.h"
#include "battery.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "cJSON.h"
#include "lwip/sockets.h"
#include "trinity_log.h"

static const char *TAG = "MOTOR_CTRL";

/* All motor GPIO pins owned here -- not in main.c */
#define MOTOR_PWM_PIN        GPIO_NUM_2
#define MOTOR_IN1_PIN        GPIO_NUM_3
#define MOTOR_IN2_PIN        GPIO_NUM_4

#define KNOB_ADC_CHANNEL     ADC_CHANNEL_0
#define PWM_TIMER            LEDC_TIMER_0
#define PWM_MODE             LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL          LEDC_CHANNEL_0
#define PWM_DUTY_RES_LEDC    LEDC_TIMER_13_BIT
#define PWM_FREQUENCY        2000
#define MOTOR_EVENT_BUF_SIZE 48

static adc_oneshot_unit_handle_t s_adc1_handle;

static int     g_aws_motor = 0;
static int     g_aws_low   = DEFAULT_AWS_LOW;
static int     g_aws_high  = DEFAULT_AWS_HIGH;
static uint8_t g_avg_temp  = DEFAULT_AVG_TEMP;
static uint16_t g_knob_adc = 0;

/* Last accepted battery reading in mV. Used to detect and reject WiFi TX
 * burst sag artifacts. Initialised to 0 -- first reading always accepted. */
static int g_batt_last_good_mv = 0;

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
    (void)gpio_set_level(MOTOR_IN1_PIN, 1);
    (void)gpio_set_level(MOTOR_IN2_PIN, 0);
}

/*----------------------------------------------------------------------------*/

void parse_tcp_json(const char *p_buf)
{
    static uint32_t update_log_count = 0;
    cJSON  *p_json  = NULL;
    cJSON  *p_low   = NULL;
    cJSON  *p_high  = NULL;
    cJSON  *p_motor = NULL;
    cJSON  *p_temp  = NULL;
    char    buf[MOTOR_EVENT_BUF_SIZE] = {0};

    p_json = cJSON_Parse(p_buf);
    if (NULL == p_json) { return; }

    p_low   = cJSON_GetObjectItem(p_json, "low");
    p_high  = cJSON_GetObjectItem(p_json, "high");
    p_motor = cJSON_GetObjectItem(p_json, "motor");
    p_temp  = cJSON_GetObjectItem(p_json, "avg_temp");

    if (NULL != p_low)  { g_aws_low  = p_low->valueint;  }
    if (NULL != p_high) { g_aws_high = p_high->valueint; }
    if (NULL != p_motor)
    {
        g_aws_motor = p_motor->valueint;
        (void)snprintf(buf, sizeof(buf), "EVENT: MOTOR=%d\n", g_aws_motor);
        trinity_log_event(buf);
    }
    if (NULL != p_temp) { g_avg_temp = (uint8_t)p_temp->valueint; }

    if (++update_log_count % LOG_THROTTLE_N == 0)
    {
        ESP_LOGI(TAG, "UPDATED: motor=%d low=%d high=%d temp=%d",
                 g_aws_motor, g_aws_low, g_aws_high, g_avg_temp);
    }

    cJSON_Delete(p_json);
}

/*----------------------------------------------------------------------------*/

void motor_task(void *p_arg)
{
    uint32_t pwm_duty      = 0;
    int      adc_raw       = 0;
    int      t             = 0;
    uint32_t last_stats_ms = 0u;
    uint32_t last_batt_ms  = 0u;
    uint32_t now_ms        = 0u;
    int      batt_mv       = -1;
    uint8_t  batt_soc      = 0u;
    char     batt_json[BATT_JSON_BUF_SIZE] = {0};
    int      sock          = -1;

    (void)p_arg;

    trinity_wdt_add();

    (void)xEventGroupWaitBits(g_wifi_eg, WIFI_CONNECTED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);

    last_stats_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_batt_ms  = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (1)
    {
        trinity_wdt_kick();

        pwm_duty = 0;

        if (0 == g_aws_motor)
        {
            (void)adc_oneshot_read(s_adc1_handle, KNOB_ADC_CHANNEL, &adc_raw);
            g_knob_adc = (uint16_t)(ADC_MAX_RAW - adc_raw);
            pwm_duty   = (g_knob_adc * PWM_DUTY_MAX) / ADC_MAX_RAW;
        }
        else
        {
            t = (int)g_avg_temp;
            if (t < g_aws_low)  { t = g_aws_low;  }
            if (t > g_aws_high) { t = g_aws_high; }
            if (g_aws_high > g_aws_low)
            {
                pwm_duty = PWM_DUTY_MAX *
                           (uint32_t)(t - g_aws_low) /
                           (uint32_t)(g_aws_high - g_aws_low);
            }
        }

        (void)ledc_set_duty(PWM_MODE, PWM_CHANNEL, pwm_duty);
        (void)ledc_update_duty(PWM_MODE, PWM_CHANNEL);

        now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if ((now_ms - last_batt_ms) >= BATT_INTERVAL_MS)
        {
            last_batt_ms = now_ms;

            batt_mv = battery_read_mv();
            if (0 < batt_mv)
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
                    batt_soc = mv_to_soc(batt_mv);

                    ESP_LOGI(TAG, "[BATT] %d mV | %d%%", batt_mv, batt_soc);

                    if (g_client_sock_valid)
                    {
                        sock = g_client_sock;
                        (void)snprintf(batt_json, sizeof(batt_json),
                                       "{\"batt_motor\":%d}", batt_soc);
                        (void)send(sock, batt_json, strlen(batt_json), 0);
                        ESP_LOGI(TAG, "[BATT] Sent to hub: %s", batt_json);
                    }
                }
            }
            else
            {
                ESP_LOGW(TAG, "[BATT] Read failed");
            }
        }

        if ((now_ms - last_stats_ms) >= STATS_INTERVAL_MS)
        {
            last_stats_ms = now_ms;
            trinity_log_heap_stats();
            trinity_log_task_stats();
        }

        vTaskDelay(pdMS_TO_TICKS(MOTOR_LOOP_MS));
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
}

/*----------------------------------------------------------------------------*/

void adc_init(adc_oneshot_unit_handle_t *p_handle)
{
    adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id  = ADC_UNIT_1 };
    adc_oneshot_chan_cfg_t      chan_cfg = { .bitwidth = ADC_BITWIDTH_12,
                                             .atten    = ADC_ATTEN_DB_6 };
    (void)adc_oneshot_new_unit(&adc_cfg, &s_adc1_handle);
    (void)adc_oneshot_config_channel(s_adc1_handle, KNOB_ADC_CHANNEL, &chan_cfg);

    if (NULL != p_handle) { *p_handle = s_adc1_handle; }
}
