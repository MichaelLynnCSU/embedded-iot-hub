#ifndef STUB_ADC_H
#define STUB_ADC_H

/*
 * Stub: zephyr/drivers/adc.h
 * battery.c uses ADC_DT_SPEC_GET_BY_IDX, adc_is_ready_dt,
 * adc_channel_setup_dt, adc_sequence_init_dt, adc_read_dt,
 * adc_raw_to_millivolts_dt, and struct adc_dt_spec / adc_sequence.
 *
 * stub_adc.c controls g_stub_adc_ready, g_stub_adc_raw, g_stub_adc_ret
 * so battery tests can inject any ADC outcome without GPIO.
 */

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

/* Controlled by stub_adc.c */
extern bool    g_stub_adc_ready;
extern int16_t g_stub_adc_raw;
extern int     g_stub_adc_ret;

struct adc_dt_spec {
    int      channel_id;
    int32_t  vref_mv;
    uint8_t  resolution;
    uint8_t  gain;
};

struct adc_sequence {
    int16_t *buffer;
    size_t   buffer_size;
};

/* DT macro -- returns a dummy spec; stub_adc.c owns the behaviour */
#define ADC_DT_SPEC_GET_BY_IDX(node, idx) \
    { .channel_id = 0, .vref_mv = 600, .resolution = 12, .gain = 6 }

/* ADC_SETTLE_US defined in battery.c -- k_busy_wait is a no-op below */

static inline bool adc_is_ready_dt(const struct adc_dt_spec *s)
{ (void)s; return g_stub_adc_ready; }

static inline int adc_channel_setup_dt(const struct adc_dt_spec *s)
{ (void)s; return g_stub_adc_ready ? 0 : -ENODEV; }

static inline void adc_sequence_init_dt(const struct adc_dt_spec *s,
                                         struct adc_sequence *seq)
{ (void)s; (void)seq; }

static inline int adc_read_dt(const struct adc_dt_spec *s,
                               struct adc_sequence *seq)
{
    (void)s;
    if (g_stub_adc_ret != 0) { return g_stub_adc_ret; }
    if (seq && seq->buffer) { *(int16_t *)seq->buffer = g_stub_adc_raw; }
    return 0;
}

/* Convert raw to millivolts using the spec's vref and resolution.
 * ceiling = vref_mv * gain_den = 600 * 6 = 3600mV, steps = 4096.
 * mv = raw * 3600 / 4096 */
static inline void adc_raw_to_millivolts_dt(const struct adc_dt_spec *s,
                                             int32_t *val_mv)
{
    (void)s;
    /* ceiling = 600mV * 6 = 3600mV, 12-bit = 4096 steps */
    *val_mv = (*val_mv * 3600) / 4096;
}

#endif /* STUB_ADC_H */
