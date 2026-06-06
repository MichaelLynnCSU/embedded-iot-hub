/******************************************************************************
 * \file    temp.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    2026-06-02
 *
 * \brief   TMP36 temperature measurement for nRF52840 temp sensor node.
 *
 * \details Pure business logic -- no Zephyr headers.
 *          Hardware access delegated to temp_hw_zephyr.c via temp_hw.h.
 *          Returns temperature in tenths of °C (int16_t).
 *          Returns TEMP_READ_ERROR on failure.
 *
 *          TMP36 transfer function:
 *          Vout (mV) = 500 + (temp_C * 10)
 *          decidegC  = Vout_mV - 500
 *
 *          ADC scaling (performed here, not in hw layer):
 *          ADC_GAIN_1_6, ADC_REF_INTERNAL (0.6V), 12-bit
 *          Full-scale = 3600 mV, max raw = 4095
 *          raw -> mV : raw * 3600 / 4095
 ******************************************************************************/

#include "temp.h"
#include "temp_hw.h"
#include "trinity_log.h"
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdio.h>

LOG_MODULE_REGISTER(temp, LOG_LEVEL_INF);

#define TEMP_BUF_SIZE        48
#define ADC_RESOLUTION       12
#define ADC_VREF_MV          600
#define ADC_GAIN_DENOM       6
#define ADC_FULL_SCALE_MV    (ADC_VREF_MV * ADC_GAIN_DENOM)  /* 3600 mV */
#define ADC_MAX_RAW          ((1 << ADC_RESOLUTION) - 1)      /* 4095    */

/** raw -> mV: raw * 3600 / 4095 */
#define RAW_TO_MV(raw)      ((int32_t)(raw) * ADC_FULL_SCALE_MV / ADC_MAX_RAW)

/** mV -> tenths of °C: TMP36 offset 500 mV, 10 mV/°C */
#define MV_TO_DECIDEGC(mv)  ((int16_t)((mv) - 500))

/*----------------------------------------------------------------------------*/

int temp_init(void)
{
    int err = temp_hw_init();
    if (0 != err)
    {
        LOG_ERR("temp hardware init failed (err=%d)", err);
    }
    return err;
}

/*----------------------------------------------------------------------------*/

int16_t temp_read_decidegc(void)
{
    int16_t  raw      = 0;
    int32_t  mv       = 0;
    int16_t  decidegc = 0;
    char     buf[TEMP_BUF_SIZE] = {0};

    int err = temp_hw_read_raw(&raw);
    if (0 != err)
    {
        LOG_ERR("ADC read failed (err=%d)", err);
        return TEMP_READ_ERROR;
    }

    mv       = RAW_TO_MV(raw);
    decidegc = MV_TO_DECIDEGC(mv);

    LOG_INF("[TEMP] raw=%d mV=%d temp=%d.%d°C",
            (int)raw, (int)mv,
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
