#include <stdbool.h>
#include <stdint.h>

/*
 * stub_adc.c
 * Controls adc_is_ready_dt(), adc_read_dt(), and adc_raw_to_millivolts_dt()
 * behaviour via three exported globals. Test setup writes these directly;
 * battery.c reads them through the inline stubs in zephyr/drivers/adc.h.
 *
 * g_stub_adc_ready -- true  => adc_is_ready_dt returns true (default)
 *                     false => battery_init() returns -1 (not ready)
 * g_stub_adc_raw   -- raw ADC count injected into adc_read_dt output buffer
 * g_stub_adc_ret   -- non-zero => adc_read_dt returns this error code
 */

bool    g_stub_adc_ready = true;
int16_t g_stub_adc_raw   = 0;
int     g_stub_adc_ret   = 0;

void stub_adc_reset(void)
{
    g_stub_adc_ready = true;
    g_stub_adc_raw   = 0;
    g_stub_adc_ret   = 0;
}
