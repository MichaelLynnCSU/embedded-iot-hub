/******************************************************************************
 * \file battery.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Battery voltage measurement interface for nRF52840 reed sensor.
 *
 * \details Provides VDD voltage reading via the nRF52840 internal SAADC
 *          VDD monitor. No external pin or resistor divider required.
 *          See battery.c for full implementation history.
 ******************************************************************************/

#ifndef INCLUDE_BATTERY_H_
#define INCLUDE_BATTERY_H_

#include <stdint.h>

/* CR2032 voltage range for SOC calculation.
 * BATT_MV_MAX: nominal full cell voltage (3.0V).
 * BATT_MV_MIN: cutoff -- cell considered dead below this level (2.0V).
 * BATT_MV_RANGE: usable window in mV. */
#define BATT_MV_MAX    3000
#define BATT_MV_MIN    2000
#define BATT_MV_RANGE  1000

/**
 * \brief  Map a raw millivolt reading to a SOC percentage (0-100).
 * \param  mv  Cell voltage in millivolts.
 * \return SOC percent clamped to [0, 100].
 */
static inline uint8_t mv_to_soc(int mv)
{
    if (mv >= BATT_MV_MAX) { return 100u; }
    if (mv <= BATT_MV_MIN) { return 0u;   }
    return (uint8_t)((mv - BATT_MV_MIN) * 100 / BATT_MV_RANGE);
}

/** \brief Initialize battery ADC channel.
 *  \return 0 on success, negative errno on failure. */
int battery_init(void);

/** \brief Read battery voltage in millivolts.
 *  \return Voltage in mV on success, -EIO on failure. */
int battery_read_mv(void);

/** \brief Read battery voltage and return SOC percent (0-100).
 *         Logs a warning and returns 0 on read failure.
 *  \return SOC percent. */
uint8_t battery_read_soc(void);

/** \brief Print battery voltage and status to console and flash log. */
void battery_print_status(void);

#endif /* INCLUDE_BATTERY_H_ */
