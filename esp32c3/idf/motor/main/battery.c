/******************************************************************************
 * \file    battery.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Battery voltage measurement for ESP32-C3 motor controller node.
 *
 * \details Reads 9V supply voltage via ADC through a fixed resistor divider.
 *          The bottom of the divider is permanently tied to GND rail --
 *          no switching GPIO is used.
 *
 *          Hardware:
 *          - ADC: ADC_UNIT_1, ADC_CHANNEL_1 (GPIO1)
 *          - Divider: Vbat (9V) → 4kΩ → ┬ → GPIO1 (ADC1_CH1)
 *                                         └ → 1kΩ → GND (always)
 *          - R1 = 4kΩ, R2 = 1kΩ
 *          - Ratio: R2/(R1+R2) = 1k/5k = 1/5
 *          - Reconstruct: adc_pin_mV × 5 = Vbat_mV
 *          - ADC: 12-bit, ADC_ATTEN_DB_12 (150-2450mV input range)
 *          - 9V fresh → ~1.94V at ADC pin → within ceiling ✓
 *          - Quiescent draw: 9V / 5kΩ = 1.8mA (negligible vs WiFi ~100mA TX)
 *
 *          Math chain (fresh battery, 1.94V at ADC pin):
 *          - adc_raw = 1944 × 4095 / 3379 = 2356 counts
 *          - pin_mv  = 2356 × 3379 / 4095 = 1944mV
 *          - vbat_mv = 1944 × 5 = 9720mV × 9500/9720 = 9500mV ✓
 *
 * \note    ADC ownership (2026-05-04):
 *          Previously battery shared ADC_UNIT_1 with the motor speed knob
 *          (ADC1_CH0/GPIO0). Knob removed as part of power saving redesign --
 *          hub now owns all temp sensing and PID control. battery_init() now
 *          creates and owns the ADC unit internally. No handle parameter.
 *
 * \note    Attenuation change (2026-04-27):
 *          Previous: ADC_ATTEN_DB_6, ceiling ~1750mV. 1.8V at ADC pin
 *          exceeded this ceiling, saturating raw=4095 every read.
 *          Changed to ADC_ATTEN_DB_12, ceiling ~3379mV (empirical).
 *          1.94V now sits comfortably mid-range.
 *
 *          Divider design vs nRF52840 smart-lock:
 *          Smart-lock uses ADC_GAIN_1_6 which sets an internal gain of 1/6
 *          inside the nRF52840 ADC hardware. The ESP32-C3 ADC has no
 *          equivalent gain stage. ADC_ATTEN_DB_12 sets input attenuation
 *          only. The only reconstruct step is the divider ratio multiply (×5).
 *          There is no gain term to account for here.
 *
 *          ADC nonlinearity correction:
 *          Correction: vbat_mv = vbat_mv * ADC_CAL_NUM / ADC_CAL_DEN
 *          Tuned 2026-04-27: multimeter reads 9500mV, ADC reports 9720mV
 *          on a fresh battery. Ratio 9500/9720 corrects downward uniformly.
 *
 *          Why always-connected (no GPIO sink):
 *          GPIO5 was previously used to sink the bottom of the divider to
 *          GND. A floating ADC node caused ESD diodes to clamp to ~3.6V,
 *          dumping excess current into the 3.3V rail. Two ESP32-C3 boards
 *          had onboard regulators damaged and USB permanently disabled.
 *          Always-connected bottom resistor eliminates this failure mode.
 ******************************************************************************/

#include "battery.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "trinity_log.h"

#define BATTERY_TAG          "BATTERY"

#define BAT_ADC_CHANNEL      ADC_CHANNEL_1   /**< GPIO1 -- ADC1_CH1             */

#define DIVIDER_RATIO_NUM    1               /**< R2 = 1kΩ                       */
#define DIVIDER_RATIO_DEN    5               /**< R1+R2 = 4kΩ+1kΩ = 5kΩ         */

#define ADC_VREF_MV          3379            /**< ADC_ATTEN_DB_12 ceiling (empirical) */
#define ADC_MAX_RAW          4095            /**< 12-bit                          */
#define ADC_SAMPLES          64             /**< fast burst -- no inter-sample delay */

/**< Empirical ADC nonlinearity correction (tuned 2026-04-27).
 *   Fresh battery: multimeter reads 9500mV, ADC reports 9720mV before
 *   correction. Ratio 9500/9720 corrects downward uniformly.
 *   Retune: measure battery terminals with multimeter, update CAL_NUM
 *   to that reading, CAL_DEN to firmware log vbat_mv before correction. */
#define ADC_CAL_NUM          9500            /**< real vbat_mv (multimeter)      */
#define ADC_CAL_DEN          9720            /**< ADC-reported vbat_mv           */

/* 9V battery status thresholds in mV */
#define VBAT_GOOD_MV         9000            /**< well charged                   */
#define VBAT_LOW_MV          8500            /**< discharging                    */
#define VBAT_CRITICAL_MV     7500            /**< brownout risk                  */

#define VBAT_BUF_SIZE        48

static adc_oneshot_unit_handle_t s_adc_handle = NULL;

/*----------------------------------------------------------------------------*/

int battery_init(void)
{
    esp_err_t err = ESP_OK;

    adc_oneshot_unit_init_cfg_t unit_cfg =
    {
        .unit_id = ADC_UNIT_1,
    };

    err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (ESP_OK != err)
    {
        ESP_LOGE(BATTERY_TAG, "ADC unit init failed (%d)", err);
        return -1;
    }

    /* ADC_ATTEN_DB_12 empirical ceiling ~3379mV.
     * 9V × (1/5 divider) = 1.8V nominal, 9.8V × (1/5) = 1.96V worst case.
     * Both within the ceiling with safe margin.
     * No gain stage on ESP32-C3 -- attenuation only. The reconstruct
     * multiply (×5) undoes the divider ratio only; no gain term needed. */
    adc_oneshot_chan_cfg_t chan_cfg =
    {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,
    };

    err = adc_oneshot_config_channel(s_adc_handle, BAT_ADC_CHANNEL, &chan_cfg);
    if (ESP_OK != err)
    {
        ESP_LOGE(BATTERY_TAG, "ADC channel config failed (%d)", err);
        return -1;
    }

    ESP_LOGI(BATTERY_TAG, "Battery init OK (ADC1_CH1/GPIO1, 4k/1k divider, DB_12)");
    return 0;
}

/*----------------------------------------------------------------------------*/

int battery_read_mv(void)
{
    int       adc_raw = 0;
    int       pin_mv  = 0;
    int       vbat_mv = 0;
    esp_err_t err     = ESP_OK;
    uint32_t  sum     = 0;

    if (NULL == s_adc_handle)
    {
        ESP_LOGE(BATTERY_TAG, "ADC handle not initialised");
        return -1;
    }

    /* Fast burst: 64 samples with no delay (~1-2ms total).
     * Captures one consistent rail state rather than spanning multiple
     * WiFi TX bursts which sag the 9V supply and cause wild swings. */
    for (int i = 0; i < ADC_SAMPLES; i++)
    {
        int sample = 0;
        err = adc_oneshot_read(s_adc_handle, BAT_ADC_CHANNEL, &sample);
        if (ESP_OK != err)
        {
            ESP_LOGE(BATTERY_TAG, "ADC read failed (%d)", err);
            return -1;
        }
        sum += (uint32_t)sample;
    }

    adc_raw = (int)(sum >> 6);

    /* Convert raw count to mV at ADC pin.
     * ADC_VREF_MV = 3379 (empirical) for ADC_ATTEN_DB_12.
     * No gain stage on ESP32-C3 -- attenuation only. */
    pin_mv = (adc_raw * ADC_VREF_MV) / ADC_MAX_RAW;

    /* Reconstruct Vbat: undo divider ratio (×5).
     * R1=4kΩ, R2=1kΩ, ratio=1/5. No gain term. */
    vbat_mv = pin_mv * DIVIDER_RATIO_DEN / DIVIDER_RATIO_NUM;

    /* Empirical nonlinearity correction (tuned 2026-04-27).
     * ADC over-reads by ~220mV -- ratio 9500/9720 corrects downward. */
    vbat_mv = (vbat_mv * ADC_CAL_NUM) / ADC_CAL_DEN;

    ESP_LOGI(BATTERY_TAG, "raw=%d pin_mv=%d vbat_mv=%d soc=%d%%",
             adc_raw, pin_mv, vbat_mv, mv_to_soc(vbat_mv));

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

    ESP_LOGI(BATTERY_TAG, "VBAT: %d mV | SOC: %d%% (%s)",
             vbat_mv, mv_to_soc(vbat_mv), status);
}
