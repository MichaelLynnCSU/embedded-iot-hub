/******************************************************************************
 * \file    battery.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Battery voltage measurement for ESP32-C3 motor controller node.
 *
 * \details Reads 9V supply voltage via ADC through a fixed resistor divider.
 *          The bottom of the divider is permanently tied to GND rail --
 *          no switching GPIO is used. Quiescent current through the 300k
 *          total divider resistance is ~27uA, negligible compared to WiFi
 *          and MCU draw.
 *
 *          Hardware:
 *          - ADC: ADC_UNIT_1, ADC_CHANNEL_1 (GPIO1)
 *          - Divider: Vbat (9V) → 100k → 100k → ┬ → GPIO1 (ADC1_CH1)
 *                                                 └ → 100k → GND (always)
 *          - R1 = 200k (two 100k in series), R2 = 100k
 *          - Ratio: R2/(R1+R2) = 100k/300k = 1/3
 *          - Reconstruct: adc_pin_mV × 3 = Vbat_mV
 *          - ADC: 12-bit, ADC_ATTEN_DB_12 (0–3.9V input range)
 *          - 9V fresh → ADC pin ~3000mV → within 3.9V max ✓
 *
 *          Why always-connected (no GPIO sink):
 *          - A floating ADC node has no DC reference and acts as an antenna
 *          - Leakage paths (ESD diodes, PCB contamination, capacitive
 *            coupling) dominate a fully isolated node producing random reads
 *          - With the bottom resistor always tied to GND the node is always
 *            defined: ADC pin sits at Vbat/3 at all times, readable anytime
 *          - 27uA quiescent draw is irrelevant vs WiFi (~100mA TX bursts)
 *            and MCU idle current (~5mA)
 *          - No boot-state risk: GPIO defaults are irrelevant, divider is
 *            always biased correctly from power-on
 *
 *          ADC nonlinearity correction:
 *          - ESP32 ADC at ADC_ATTEN_DB_12 is nonlinear, especially near
 *            full scale. Empirical calibration against a known 9V supply
 *            showed ~18% overread (10600mV reported vs 9000mV real).
 *          - Correction: vbat_mv = vbat_mv * ADC_CAL_NUM / ADC_CAL_DEN
 *          - ADC_CAL_NUM = 9000  (multimeter measured)
 *          - ADC_CAL_DEN = 10600 (ADC reported before correction)
 *          - Retune by measuring battery with multimeter and updating
 *            ADC_CAL_NUM to the real voltage, ADC_CAL_DEN to what the
 *            ADC reports before correction.
 *
 *          Averaging:
 *          - 64 samples taken as fast as possible (~1-2ms total burst)
 *            rather than spread across a long window. A wide sample window
 *            (e.g. 16 samples × 5ms = 80ms) catches multiple WiFi TX
 *            bursts which genuinely sag the 9V rail, producing wild swings.
 *            A fast burst captures one consistent rail state and averages
 *            out ADC quantisation noise without spanning a TX event.
 *
 *          Thresholds (9V regulated supply):
 *          - GOOD:     >= 8500mV  (well regulated)
 *          - LOW:      >= 8000mV  (supply sagging)
 *          - CRITICAL: >= 7500mV  (brownout risk)
 *          - DEAD:      < 7500mV  (unreliable operation)
 *
 * \note    ADC channel conflict:
 *          ADC1_CH0 (GPIO0) is already used for the motor speed knob.
 *          Battery uses ADC1_CH1 (GPIO1) to avoid conflict. Both channels
 *          share the same adc_oneshot unit handle (g_adc1_handle) initialised
 *          in motor_control.c -- battery_init() only configures the
 *          additional channel, it does not create a new unit.
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
 ******************************************************************************/

#include "battery.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "trinity_log.h"

#define BATTERY_TAG          "BATTERY"

#define BAT_ADC_CHANNEL      ADC_CHANNEL_1   /**< GPIO1 -- ADC1_CH1            */

#define DIVIDER_RATIO_NUM    1               /**< R2 = 100k                     */
#define DIVIDER_RATIO_DEN    3               /**< R1+R2 = 200k+100k = 300k      */

#define ADC_VREF_MV          3900            /**< ADC_ATTEN_DB_12 full scale    */
#define ADC_MAX_RAW          4095            /**< 12-bit                        */
#define ADC_SAMPLES          64             /**< fast burst -- no inter-sample  */
                                            /**< delay, captures one rail state */

/**< Empirical ADC nonlinearity correction.
 *   Measured: 9000mV real (multimeter), 10600mV reported (ADC).
 *   To retune: measure battery with multimeter, update CAL_NUM to real mV,
 *   update CAL_DEN to what ADC reports before correction is applied.       */
#define ADC_CAL_NUM          9000
#define ADC_CAL_DEN          10600

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

    ESP_LOGI(BATTERY_TAG, "Battery init OK (ADC1_CH1/GPIO1, always-on divider)");

    return 0;
}

/*----------------------------------------------------------------------------*/

int battery_read_mv(void)
{
    int       adc_raw    = 0;
    int       pin_mv     = 0;
    int       vbat_mv    = 0;
    esp_err_t err        = ESP_OK;
    uint32_t  sum        = 0;

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

    /* Divide by 64 using shift for efficiency */
    adc_raw = (int)(sum >> 6);

    /* Convert raw to mV at ADC pin (linear approximation) */
    pin_mv = (adc_raw * ADC_VREF_MV) / ADC_MAX_RAW;

    /* Reconstruct Vbat: ADC pin sees Vbat/3, multiply back by 3 */
    vbat_mv = pin_mv * DIVIDER_RATIO_DEN / DIVIDER_RATIO_NUM;

    /* Empirical ADC nonlinearity correction (see file header for tuning) */
    vbat_mv = (vbat_mv * ADC_CAL_NUM) / ADC_CAL_DEN;

    ESP_LOGI(BATTERY_TAG, "raw=%d pin_mv=%d vbat_mv=%d", adc_raw, pin_mv, vbat_mv);

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
