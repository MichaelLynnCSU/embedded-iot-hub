/******************************************************************************
 * \file    battery.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Battery voltage measurement for ESP32-C3 motor controller node.
 *
 * \details Reads 9V supply voltage via ADC through a switched resistor
 *          divider. GPIO5 sinks the bottom of the divider to GND to enable
 *          measurement, eliminating quiescent drain during idle periods.
 *
 *          Hardware:
 *          - ADC: ADC_UNIT_1, ADC_CHANNEL_1 (GPIO1)
 *          - Divider: Vbat (9V) → 100k → 100k → ┬ → GPIO1 (ADC1_CH1)
 *                                                 └ → 100k → GPIO5 (LOW=GND)
 *          - R1 = 200k (two 100k in series), R2 = 100k
 *          - Ratio: R2/(R1+R2) = 100k/300k = 1/3
 *          - Reconstruct: adc_pin_mV × 3 = Vbat_mV
 *          - ADC: 12-bit, ADC_ATTEN_DB_12 (0–3.9V input range)
 *          - 9V fresh → ADC pin ~3000mV → within 3.9V max ✓
 *          - GPIO5: drive LOW to enable (GND sink), drive HIGH to disable
 *
 *          GPIO5 voltage when enabled:
 *          - GPIO5 is driven to 0V (GND sink) -- no overvoltage risk
 *          - GPIO1 sees Vbat × 100k/300k = Vbat/3 ≈ 3000mV at 9V ✓
 *          - Well within ESP32-C3 IO limit of 3.3V and ADC max of 3.9V
 *
 *          Thresholds (9V regulated supply):
 *          - GOOD:     >= 8500mV  (well regulated)
 *          - LOW:      >= 8000mV  (supply sagging)
 *          - CRITICAL: >= 7500mV  (brownout risk)
 *          - DEAD:      < 7500mV  (unreliable operation)
 *
 * \note    Why low-side GPIO sink instead of high-side:
 *          Previous design wired GPIO5 into the high-voltage side of the
 *          divider (Vbat → GPIO5 → resistors → GPIO1). With a 9V supply
 *          this applied 9V to an IO pin rated for 3.3V max. The ESD diodes
 *          clamped into the 3.3V rail, damaging the onboard regulator and
 *          permanently disabling USB on two ESP32-C3 boards.
 *          Fix: GPIO5 now sinks the bottom of R2 to GND (low-side switch).
 *          When enabled GPIO5 is at 0V -- safe regardless of supply voltage.
 *
 * \note    ADC channel conflict:
 *          ADC1_CH0 (GPIO0) is already used for the motor speed knob.
 *          Battery uses ADC1_CH1 (GPIO1) to avoid conflict. Both channels
 *          share the same adc_oneshot unit handle (g_adc1_handle) initialised
 *          in motor_control.c -- battery_init() only configures the
 *          additional channel, it does not create a new unit.
 ******************************************************************************/

#include "battery.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "trinity_log.h"

#define BATTERY_TAG          "BATTERY"

#define BAT_ADC_CHANNEL      ADC_CHANNEL_3   /**< GPIO1 -- ADC1_CH1            */
#define BAT_SINK_PIN         GPIO_NUM_7      /**< GPIO5 -- low-side GND sink    */

#define DIVIDER_RATIO_NUM    1               /**< R2 = 100k                     */
#define DIVIDER_RATIO_DEN    3               /**< R1+R2 = 200k+100k = 300k     */

#define ADC_VREF_MV          3900            /**< ADC_ATTEN_DB_12 full scale    */
#define ADC_MAX_RAW          4095            /**< 12-bit                        */

#define ADC_SETTLE_MS        500              /**< settle through 300k source    */

/* 9V regulated supply thresholds in mV */
#define VBAT_GOOD_MV         8500            /**< well regulated                */
#define VBAT_LOW_MV          8000            /**< supply sagging                */
#define VBAT_CRITICAL_MV     7500            /**< brownout risk                 */

#define VBAT_BUF_SIZE        48

static adc_oneshot_unit_handle_t s_adc_handle = NULL;

/*----------------------------------------------------------------------------*/

int battery_init(adc_oneshot_unit_handle_t adc_handle)
{
    esp_err_t err = ESP_OK;

    if (NULL == adc_handle)
    {
        ESP_LOGE(BATTERY_TAG, "ADC handle is NULL");
        return -1;
    }

    s_adc_handle = adc_handle;

    /* Configure battery ADC channel -- shares unit with knob (CH0) */
    adc_oneshot_chan_cfg_t chan_cfg =
    {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,   /* 0-3.9V input range */
    };

    err = adc_oneshot_config_channel(s_adc_handle, BAT_ADC_CHANNEL, &chan_cfg);
    if (ESP_OK != err)
    {
        ESP_LOGE(BATTERY_TAG, "ADC channel config failed (%d)", err);
        return -1;
    }

    /* Sink pin HIGH -- divider disabled, no quiescent drain */
    (void)gpio_set_direction(BAT_SINK_PIN, GPIO_MODE_OUTPUT);
    (void)gpio_set_level(BAT_SINK_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));   // long settle for testing

    ESP_LOGI(BATTERY_TAG,
    "Battery init OK (9V supply, GPIO3)");
          return 0;
}

/*----------------------------------------------------------------------------*/

int battery_read_mv(void)
{
    int       adc_raw    = 0;
    int       sink_level = 0;
    int       pin_mv     = 0;
    int       vbat_mv    = 0;
    esp_err_t err        = ESP_OK;

    if (NULL == s_adc_handle)
    {
        ESP_LOGE(BATTERY_TAG, "ADC handle not initialised");
        return -1;
    }

    /* Drive LOW -- GPIO5 sinks bottom of divider to GND, current flows */
    (void)gpio_set_level(BAT_SINK_PIN, 0);
     vTaskDelay(pdMS_TO_TICKS(100));
     int sink = gpio_get_level(BAT_SINK_PIN);
     int adc_pin = gpio_get_level(GPIO_NUM_0);
     ESP_LOGI(BATTERY_TAG, "sink_level=%d adc_gpio_level=%d", sink, adc_pin);

    /* 10ms settle: allow ADC input cap to charge through 300k source */
    vTaskDelay(pdMS_TO_TICKS(ADC_SETTLE_MS));

    /* Read sink level DURING measurement -- before driving high again */
    sink_level = gpio_get_level(BAT_SINK_PIN);

    err = adc_oneshot_read(s_adc_handle, BAT_ADC_CHANNEL, &adc_raw);

    /* Drive HIGH -- no current path, zero quiescent drain */
    (void)gpio_set_level(BAT_SINK_PIN, 1);

    if (ESP_OK != err)
    {
        ESP_LOGE(BATTERY_TAG, "ADC read failed (%d)", err);
        return -1;
    }

    /* Convert raw to mV at ADC pin (linear approximation) */
    pin_mv = (adc_raw * ADC_VREF_MV) / ADC_MAX_RAW;

    /* Reconstruct Vbat: ADC pin sees Vbat/3, multiply back by 3 */
    vbat_mv = pin_mv * DIVIDER_RATIO_DEN / DIVIDER_RATIO_NUM;

    ESP_LOGI(BATTERY_TAG,
             "sink=%d raw=%d pin_mv=%d vbat_mv=%d",
             sink_level, adc_raw, pin_mv, vbat_mv);

    return vbat_mv;
}

/*----------------------------------------------------------------------------*/

void battery_print_status(void)
{
    char        buf[VBAT_BUF_SIZE] = {0};
    const char *status             = NULL;

    int vbat_mv = battery_read_mv();
    if (0 > vbat_mv)
    {
        ESP_LOGE(BATTERY_TAG, "Battery read failed");
        return;
    }

    (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | VBAT: %d mV\n", vbat_mv);
    trinity_log_event(buf);

    if      (vbat_mv >= VBAT_GOOD_MV)     { status = "GOOD";     }
    else if (vbat_mv >= VBAT_LOW_MV)      { status = "LOW";      }
    else if (vbat_mv >= VBAT_CRITICAL_MV) { status = "CRITICAL"; }
    else                                   { status = "DEAD";     }

    ESP_LOGI(BATTERY_TAG, "VBAT: %d mV (%s)", vbat_mv, status);
}
