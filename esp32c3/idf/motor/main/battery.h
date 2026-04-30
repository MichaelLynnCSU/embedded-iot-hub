/******************************************************************************
 * \file    battery.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Battery voltage measurement API for ESP32-C3 motor controller.
 *
 * \note    SOC range (2026-04-27):
 *          VBAT_FULL_MV = 9500mV -- empirical: multimeter reads 9500mV at
 *          battery terminals on a fresh charge. ADC changed to
 *          ADC_ATTEN_DB_11 -- 1.8V at ADC pin was saturating the DB_6
 *          ceiling (~1750mV), producing raw=4095 on every read. DB_11
 *          ceiling is ~2450mV; 1.8V sits comfortably mid-range.
 *          Cal reset to 1/1 -- retune after attenuation change.
 *          VBAT_DEAD_MV = 7500mV -- existing CRITICAL threshold below which
 *          motor operation is unreliable.
 *          VBAT_RANGE_MV = VBAT_FULL_MV - VBAT_DEAD_MV = 2000mV.
 ******************************************************************************/

#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include "esp_adc/adc_oneshot.h"

/* ---- SOC thresholds (real-world mV at battery terminals) ---- */
#define VBAT_FULL_MV         9500            /**< fresh battery, 9.5V at terminals */
#define VBAT_DEAD_MV         7500            /**< below this motor is unreliable   */
#define VBAT_RANGE_MV        (VBAT_FULL_MV - VBAT_DEAD_MV)  /**< 2000mV          */

/**
 * \brief  Convert battery voltage in mV to state-of-charge percent (0-100).
 *
 *         Linear interpolation between VBAT_DEAD_MV (0%) and VBAT_FULL_MV
 *         (100%). Clamps at both ends. Matches mv_to_soc() on the nRF52840
 *         smart-lock -- hub can treat both nodes identically.
 *
 * \param  vbat_mv  Battery voltage in mV (post cal-correction)
 * \return SOC 0-100
 */
static inline uint8_t mv_to_soc(int vbat_mv)
{
    if (vbat_mv >= VBAT_FULL_MV) { return 100u; }
    if (vbat_mv <= VBAT_DEAD_MV) { return 0u;   }
    return (uint8_t)((vbat_mv - VBAT_DEAD_MV) * 100 / VBAT_RANGE_MV);
}

/**
 * \brief  Initialise battery ADC channel.
 *
 * \note   Shares the ADC unit handle created in app_main (g_adc1_handle).
 *         Call after adc_init() in app_main, passing g_adc1_handle.
 *
 * \param  adc_handle  Existing ADC_UNIT_1 handle
 * \return 0 on success, -1 on error
 */
int battery_init(adc_oneshot_unit_handle_t adc_handle);

/**
 * \brief  Read battery voltage in millivolts.
 *
 *         Reads ADC1_CH1 (GPIO1) through a 4k/1k resistor divider
 *         (ratio 1/5) permanently tied to GND. Reconstructs Vbat
 *         by multiplying the ADC pin voltage by 5.
 *
 * \return Vbat in mV, or -1 on error
 */
int battery_read_mv(void);

/**
 * \brief  Read and log battery status (GOOD / LOW / CRITICAL / DEAD).
 *         Logs to trinity_log and ESP_LOGI.
 */
void battery_print_status(void);

#endif /* BATTERY_H */
