/******************************************************************************
 * \file    temp.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    2026-06-02
 *
 * \brief   TMP36 temperature measurement for nRF52840 temp sensor node.
 *
 * \details Reads TMP36 output via SAADC channel 0 (AIN0, P0.02).
 *          Uses Zephyr ADC API. Returns temperature in tenths of °C
 *          (int16_t). Returns TEMP_READ_ERROR on failure so callers
 *          can detect and skip bad readings.
 *
 *          Hardware:
 *          - Board:    Teyleten Robot Pro Micro nRF52840
 *          - TMP36 OUT → P0.02 (AIN0), SAADC channel 0
 *          - VDD rail: 2.5V (HX3001 LDO — see battery.c hardware notes)
 *          - ADC:      ADC_GAIN_1_6, ADC_REF_INTERNAL (0.6V), 12-bit
 *                      Full-scale input = 0.6 * 6 = 3.6V
 *                      Resolution       = 3600 mV / 4096 = 0.879 mV/LSB
 *
 * \note    HARDWARE PINOUT VERIFICATION:
 *          Channel 1 (AIN1 / P0.03) does not exist on the Teyleten Robot Pro 
 *          Micro nRF52840 pinout headers. Shifted TMP36 output routing to 
 *          physical pin A0 (AIN0 / P0.02) on channel 0. Because the designers 
 *          broke out P0.02 (A0), P0.29 (A1), and P0.31 (A2), they still 
 *          successfully provided you with three distinct analog input channels 
 *          (Channels 0, 5, and 7).
 *
 * \note    TMP36 transfer function:
 *          Vout (mV) = 500 + (temp_C * 10)
 *          decidegC  = Vout_mV - 500
 *
 * \note    Why SAADC is safe here (unlike battery):
 *          TMP36 output impedance is ~1 ohm. The SAADC input capacitance
 *          charge current that collapsed the CR2032 in battery Tests 2/4
 *          is trivially sourced from the TMP36 op-amp output stage.
 *          No bulk capacitors needed. Direct AIN connection is correct.
 *
 * \note    Acquisition time:
 *          ADC_ACQ_TIME_DEFAULT (10 us) is sufficient for a 1-ohm source.
 *          The settling time formula: t = R * C_in * 10 = 1 * ~5pF * 10
 *          = ~50ps. 10us is orders of magnitude above the minimum.
 ******************************************************************************/

#include "temp.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_saadc.h>
#include "trinity_log.h"

LOG_MODULE_REGISTER(temp, LOG_LEVEL_INF);

#define TEMP_BUF_SIZE        48     /**< log message buffer size          */
#define ADC_RESOLUTION       12     /**< SAADC resolution bits            */
#define ADC_VREF_MV          600    /**< internal reference voltage mV    */
#define ADC_GAIN_DENOM       6      /**< gain denominator (ADC_GAIN_1_6)  */
#define ADC_FULL_SCALE_MV    (ADC_VREF_MV * ADC_GAIN_DENOM)  /* 3600 mV */
#define ADC_MAX_RAW          ((1 << ADC_RESOLUTION) - 1)      /* 4095    */

/** raw → mV: raw * 3600 / 4095 */
#define RAW_TO_MV(raw)  ((int32_t)(raw) * ADC_FULL_SCALE_MV / ADC_MAX_RAW)

/** mV → tenths of °C: TMP36 offset 500 mV, 10 mV/°C */
#define MV_TO_DECIDEGC(mv)  ((int16_t)((mv) - 500))

static const struct device *g_adc = NULL;

static const struct adc_channel_cfg g_ch1_cfg = {
    .gain             = ADC_GAIN_1_6,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 0,
    .input_positive   = NRF_SAADC_INPUT_AIN0,  /* P0.02 */
};

static int16_t g_sample_buf;

static const struct adc_sequence g_seq = {
    .channels    = BIT(0),
    .buffer      = &g_sample_buf,
    .buffer_size = sizeof(g_sample_buf),
    .resolution  = ADC_RESOLUTION,
};

/*----------------------------------------------------------------------------*/

int temp_init(void)
{
    int err = 0;

    g_adc = DEVICE_DT_GET(DT_NODELABEL(adc));

    if (NULL == g_adc)
    {
        LOG_ERR("ADC device not found");
        return -ENODEV;
    }

    if (!device_is_ready(g_adc))
    {
        LOG_ERR("ADC not ready");
        g_adc = NULL;
        return -ENODEV;
    }

    err = adc_channel_setup(g_adc, &g_ch1_cfg);
    if (0 != err)
    {
        LOG_ERR("ADC channel 1 setup failed (err=%d)", err);
        g_adc = NULL;
        return err;
    }

    LOG_INF("TMP36 ADC channel 0 ready (P0.02 / AIN0)");
    return 0;
}

/*----------------------------------------------------------------------------*/

int16_t temp_read_decidegc(void)
{
    int      err = 0;
    int32_t  mv  = 0;
    char     buf[TEMP_BUF_SIZE] = {0};
    int16_t  decidegc = 0;

    if (NULL == g_adc) { return TEMP_READ_ERROR; }

    err = adc_read(g_adc, &g_seq);
    if (0 != err)
    {
        LOG_ERR("ADC read failed (err=%d)", err);
        return TEMP_READ_ERROR;
    }

    mv       = RAW_TO_MV(g_sample_buf);
    decidegc = MV_TO_DECIDEGC(mv);

    LOG_INF("[TEMP] raw=%d mV=%d temp=%d.%d°C",
            g_sample_buf, (int)mv,
            (int)(decidegc / 10),
            (int)(decidegc < 0 ? -(decidegc % 10) : decidegc % 10));

    (void)snprintf(buf, sizeof(buf), "EVENT: TEMP %d.%d C\n",
                   (int)(decidegc / 10),
                   (int)(decidegc < 0 ? -(decidegc % 10) : decidegc % 10));
    trinity_log_event(buf);

    return decidegc;
}

/*----------------------------------------------------------------------------*/

void temp_print_status(void)
{
    int16_t t = temp_read_decidegc();

    if (TEMP_READ_ERROR == t)
    {
        LOG_ERR("[TEMP] Read failed");
        return;
    }

    LOG_INF("[TEMP] %d.%d°C",
            (int)(t / 10),
            (int)(t < 0 ? -(t % 10) : t % 10));
}
