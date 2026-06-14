/******************************************************************************
 * \file buzzer.c
 * \brief Passive buzzer driver — ESP32 doorbell.
 *
 * \details LEDC_TIMER_1 / LEDC_CHANNEL_1 reserved for buzzer.
 *          Camera uses LEDC_TIMER_0 / LEDC_CHANNEL_0 for XCLK — do not
 *          change the timer/channel assignments here without checking
 *          cam_logic.h and camera_config_t in main.c.
 *
 *          GPIO14 → 1kΩ → 2N7000 gate
 *                          2N7000 drain → buzzer-
 *                          2N7000 source → GND
 *                          buzzer+ → VCC
 *                          1N4148 across buzzer (cathode toward VCC)
 ******************************************************************************/

#include "buzzer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define BUZZER_GPIO         GPIO_NUM_14
#define BUZZER_LEDC_TIMER   LEDC_TIMER_1
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_1
#define BUZZER_FREQ_HZ      2000u    /**< 2kHz tone — audible, not shrill   */
#define BUZZER_DUTY_RES     LEDC_TIMER_10_BIT
#define BUZZER_DUTY_50PCT   512u     /**< 50% of 2^10 — maximum volume      */
#define BUZZER_DURATION_MS  200u     /**< beep length in ms                 */

static const char *TAG = "BUZZER";

void buzzer_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = BUZZER_LEDC_TIMER,
        .duty_resolution = BUZZER_DUTY_RES,
        .freq_hz         = BUZZER_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "ledc_timer_config failed: 0x%x", err);
        return;
    }

    ledc_channel_config_t channel = {
        .gpio_num   = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = BUZZER_LEDC_CHANNEL,
        .timer_sel  = BUZZER_LEDC_TIMER,
        .duty       = 0,    /* start silent */
        .hpoint     = 0,
    };
    err = ledc_channel_config(&channel);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "ledc_channel_config failed: 0x%x", err);
        return;
    }

    ESP_LOGI(TAG, "Buzzer init OK (GPIO%d, %uHz, LEDC_TIMER_%d CH_%d)",
             BUZZER_GPIO, BUZZER_FREQ_HZ,
             BUZZER_LEDC_TIMER, BUZZER_LEDC_CHANNEL);
}

void buzzer_beep(void)
{
    /* On */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY_50PCT);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);

    vTaskDelay(pdMS_TO_TICKS(BUZZER_DURATION_MS));

    /* Off */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);

    ESP_LOGI(TAG, "Beep (%ums)", BUZZER_DURATION_MS);
}
