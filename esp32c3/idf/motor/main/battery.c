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
 *          - ADC: 12-bit, ADC_ATTEN_DB_12 (150–2450mV input range)
 *          - 9V fresh → ~1.94V at ADC pin → within ceiling ✓
 *          - Quiescent draw: 9V / 5kΩ = 1.8mA (negligible vs WiFi ~100mA TX)
 *
 *          Math chain (fresh battery, 1.94V at ADC pin):
 *          - adc_raw = 1944 × 4095 / 3379 = 2356 counts
 *          - pin_mv  = 2356 × 3379 / 4095 = 1944mV
 *          - vbat_mv = 1944 × 5 = 9720mV × 9500/9720 = 9500mV ✓
 *
 *          Attenuation change (2026-04-27):
 *          - Previous: ADC_ATTEN_DB_6, ceiling ~1750mV. 1.8V at ADC pin
 *            exceeded this ceiling, saturating raw=4095 every read.
 *          - Changed to ADC_ATTEN_DB_12, ceiling ~3379mV (empirical).
 *            1.94V now sits comfortably mid-range.
 *          - ADC_VREF_MV updated from 2200 to 3379 (back-calculated from
 *            known pin voltage: 1800 × 4095 / 2181 = 3379mV).
 *
 *          Divider design vs nRF52840 smart-lock:
 *          - Smart-lock uses ADC_GAIN_1_6 which sets an internal gain of 1/6
 *            inside the nRF52840 ADC hardware. This scales the full-scale
 *            ceiling to ref × gain_den = 600mV × 6 = 3600mV. The divider
 *            ratio on the nRF must account for this gain when choosing
 *            resistor values; the reconstruct multiply must use DIVIDER_RATIO_DEN
 *            only, not the gain, because adc_raw_to_millivolts_dt() already
 *            folds the gain into the millivolt result.
 *          - The ESP32-C3 ADC has no equivalent gain stage. ADC_ATTEN_DB_12
 *            sets the input attenuation to extend the measurable range;
 *            it does not introduce a multiplier that must be undone in
 *            firmware. adc_oneshot_read() returns a raw count; the linear
 *            conversion (raw × VREF / 4095) gives pin millivolts directly.
 *            The only reconstruct step needed is the divider ratio multiply
 *            (× 5). There is no gain term to account for here.
 *
 *          Why always-connected (no GPIO sink):
 *          - A floating ADC node has no DC reference and acts as an antenna
 *          - Leakage paths (ESD diodes, PCB contamination, capacitive
 *            coupling) dominate a fully isolated node producing random reads
 *          - With the bottom resistor always tied to GND the node is always
 *            defined: ADC pin sits at Vbat/5 at all times, readable anytime
 *          - 1.8mA quiescent draw is irrelevant vs WiFi (~100mA TX bursts)
 *            and MCU idle current (~5mA)
 *          - No boot-state risk: GPIO defaults are irrelevant, divider is
 *            always biased correctly from power-on
 *
 *          ADC nonlinearity correction:
 *          - ESP32 ADC is nonlinear, especially near full scale. Empirical
 *            calibration against a known supply is required after bring-up.
 *          - Correction: vbat_mv = vbat_mv * ADC_CAL_NUM / ADC_CAL_DEN
 *          - ADC_CAL_NUM = real vbat_mv (multimeter at battery terminals)
 *          - ADC_CAL_DEN = ADC-reported vbat_mv before correction
 *          - Retune: measure battery with multimeter, update CAL_NUM to real
 *            voltage, CAL_DEN to what firmware logs as vbat_mv before
 *            correction.
 *          - Tuned 2026-04-27: multimeter reads 9500mV, ADC reports 9720mV
 *            on a fresh battery. Ratio 9500/9720 corrects downward uniformly.
 *
 *          Averaging:
 *          - 64 samples taken as fast as possible (~1-2ms total burst)
 *            rather than spread across a long window. A wide sample window
 *            catches multiple WiFi TX bursts which genuinely sag the 9V
 *            rail, producing wild swings. A fast burst captures one
 *            consistent rail state and averages out ADC quantisation noise
 *            without spanning a TX event.
 *
 * \note    ADC channel conflict:
 *          ADC1_CH0 (GPIO0) is already used for the motor speed knob.
 *          Battery uses ADC1_CH1 (GPIO1) to avoid conflict. Both channels
 *          share the same adc_oneshot unit handle (g_adc1_handle) initialised
 *          in app_main.c -- battery_init() only configures the additional
 *          channel, it does not create a new unit.
 *
 * \note    Previous design (now removed):
 *          GPIO5 was used to sink the bottom of the divider to GND to
 *          eliminate quiescent drain. This caused a floating ADC node
 *          whenever GPIO5 was HIGH (disabled), pulling the ADC pin toward
 *          the 9V rail through the top resistors. ESD diodes clamped the
 *          pin to ~3.6V and dumped excess current into the 3.3V rail.
 *          Two ESP32-C3 boards had their onboard regulators damaged and
 *          USB permanently disabled as a result. Always-connected bottom
 *          resistor eliminates this failure mode entirely.
 *
 * \note    Divider resistors changed (2026-04-27):
 *          Previous design used three 100kΩ resistors (R1=200k, R2=100k,
 *          ratio=1/3). With a 9V input this produced 3.0V at the ADC pin,
 *          exceeding the ADC_ATTEN_DB_6 ceiling of 2.2V. Resistors changed
 *          to R1=4kΩ, R2=1kΩ (ratio=1/5). 9V × 1/5 = 1.8V -- safely within
 *          the ceiling. Attenuation changed from ADC_ATTEN_DB_6 to
 *          ADC_ATTEN_DB_12. ADC_VREF_MV updated to 3379 (empirical).
 *          Calibration tuned to 9500/9720.
 ******************************************************************************/

#include "battery.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "trinity_log.h"

#define BATTERY_TAG          "BATTERY"

#define BAT_ADC_CHANNEL      ADC_CHANNEL_1   /**< GPIO1 -- ADC1_CH1             */

#define DIVIDER_RATIO_NUM    1               /**< R2 = 1kΩ                       */
#define DIVIDER_RATIO_DEN    5               /**< R1+R2 = 4kΩ+1kΩ = 5kΩ         */

#define ADC_VREF_MV          3379            /**< ADC_ATTEN_DB_12 ceiling (empirical) */
#define ADC_MAX_RAW          4095            /**< 12-bit                          */
#define ADC_SAMPLES          64             /**< fast burst -- no inter-sample    */
                                            /**< delay, captures one rail state   */

/**< Empirical ADC nonlinearity correction (tuned 2026-04-27).
 *   Fresh battery: multimeter reads 9500mV at terminals, ADC reports
 *   9720mV before correction. Ratio 9500/9720 corrects downward uniformly.
 *   Retune by: measure battery terminals with multimeter, update CAL_NUM
 *   to that reading, CAL_DEN to what firmware logs as vbat_mv before
 *   correction. */
#define ADC_CAL_NUM          9500            /**< real vbat_mv (multimeter)      */
#define ADC_CAL_DEN          9720            /**< ADC-reported vbat_mv           */

/* 9V battery status thresholds in mV */
#define VBAT_GOOD_MV         9000            /**< well charged                   */
#define VBAT_LOW_MV          8500            /**< discharging                    */
#define VBAT_CRITICAL_MV     7500            /**< brownout risk -- same as        */
                                             /**< VBAT_DEAD_MV in battery.h      */

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

    /* Configure battery ADC channel -- shares unit with knob (CH0).
     * ADC_ATTEN_DB_12 empirical ceiling ~3379mV.
     * 9V × (1/5 divider) = 1.8V nominal, 9.8V × (1/5) = 1.96V worst case.
     * Both within the ceiling with safe margin.
     * Previous ADC_ATTEN_DB_6 (ceiling ~1750mV) saturated at 1.8V pin --
     * changed to DB_12 to bring pin voltage into mid-range.
     * No gain stage exists on the ESP32-C3 ADC -- attenuation only.
     * The reconstruct multiply (× 5) undoes the divider ratio only;
     * there is no additional gain term as there is on the nRF52840. */
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

    /* Fast burst: 64 samples with no delay between them (~1-2ms total).
     * Captures one consistent rail state rather than spanning multiple
     * WiFi TX bursts which sag the 9V supply and cause wild swings.     */
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
     * No gain stage on ESP32-C3 -- attenuation only. Compare: nRF52840
     * ADC_GAIN_1_6 folds a ×6 ceiling multiplier into adc_raw_to_millivolts_dt()
     * producing a 3600mV ceiling. Here there is no such term -- pin_mv is
     * the direct linear result of raw × ceiling / steps. */
    pin_mv = (adc_raw * ADC_VREF_MV) / ADC_MAX_RAW;

    /* Reconstruct Vbat: undo divider ratio (×5).
     * R1=4kΩ, R2=1kΩ, ratio=1/5. No gain term. */
    vbat_mv = pin_mv * DIVIDER_RATIO_DEN / DIVIDER_RATIO_NUM;

    /* Empirical nonlinearity correction (tuned 2026-04-27).
     * ADC over-reads by ~220mV on this hardware -- ratio 9500/9720
     * corrects downward uniformly. See ADC_CAL_NUM/ADC_CAL_DEN defines. */
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
