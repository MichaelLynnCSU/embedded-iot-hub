#include "temp_hw.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_saadc.h>

LOG_MODULE_DECLARE(temp, LOG_LEVEL_INF);

#define ADC_RESOLUTION    12
#define ADC_VREF_MV       600
#define ADC_GAIN_DENOM    6

static const struct device *g_adc_hw = NULL;

static const struct adc_channel_cfg g_ch_cfg = {
    .gain             = ADC_GAIN_1_6,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 0,
    .input_positive   = NRF_SAADC_INPUT_AIN0,
};

static int16_t g_sample_buf;

static const struct adc_sequence g_seq = {
    .channels    = BIT(0),
    .buffer      = &g_sample_buf,
    .buffer_size = sizeof(g_sample_buf),
    .resolution  = ADC_RESOLUTION,
};

int temp_hw_init(void)
{
    g_adc_hw = DEVICE_DT_GET(DT_NODELABEL(adc));
    if (!g_adc_hw || !device_is_ready(g_adc_hw)) {
        g_adc_hw = NULL;
        return -ENODEV;
    }
    return adc_channel_setup(g_adc_hw, &g_ch_cfg);
}

int temp_hw_read_raw(int16_t *out_raw)
{
    if (!g_adc_hw) return -ENODEV;
    int err = adc_read(g_adc_hw, &g_seq);
    if (err) return err;
    *out_raw = g_sample_buf;
    return 0;
}
