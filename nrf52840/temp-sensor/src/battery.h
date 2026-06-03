/******************************************************************************
 * \file battery.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Battery measurement interface for nRF52840 reed sensor.
 *
 * \details SOC and voltage via MAX17048 fuel gauge over I2C1.
 *          See battery.c for full implementation and debugging history.
 *
 * \note    mv_to_soc() retained for fallback to Test 1 (resistor divider)
 *          if MAX17048 hardware becomes unavailable. Do not remove.
 ******************************************************************************/

#ifndef INCLUDE_BATTERY_H_
#define INCLUDE_BATTERY_H_

#include <stdint.h>

/* CR2032 voltage range for SOC calculation -- used by mv_to_soc() fallback.
 * BATT_MV_MAX: nominal full cell voltage (3.0V).
 * BATT_MV_MIN: cutoff -- cell considered dead below this level (2.0V).
 * BATT_MV_RANGE: usable window in mV. */
#define BATT_MV_MAX    3000
#define BATT_MV_MIN    2000
#define BATT_MV_RANGE  1000

/**
 * \brief  Map a raw millivolt reading to a SOC percentage (0-100).
 *         Fallback for Test 1 resistor divider approach if MAX17048
 *         hardware is unavailable. Not used in current implementation.
 * \param  mv  Cell voltage in millivolts.
 * \return SOC percent clamped to [0, 100].
 */
static inline uint8_t mv_to_soc(int mv)
{
    if (mv >= BATT_MV_MAX) { return 100u; }
    if (mv <= BATT_MV_MIN) { return 0u;   }
    return (uint8_t)((mv - BATT_MV_MIN) * 100 / BATT_MV_RANGE);
}

/** \brief Initialize MAX17048 fuel gauge via I2C1.
 *  \return 0 on success, negative errno on failure. */
int battery_init(void);

/** \brief Read battery voltage in millivolts from MAX17048.
 *  \return Voltage in mV on success, -EIO on failure. */
int battery_read_mv(void);

/** \brief Read battery SOC percent (0-100) from MAX17048.
 *         Clamped to 100 -- ModelGauge reports >100 on fresh cells.
 *         Logs warning and returns 0 on read failure.
 *  \return SOC percent. */
uint8_t battery_read_soc(void);

/** \brief Print battery voltage and status to console and flash log. */
void battery_print_status(void);

#endif /* INCLUDE_BATTERY_H_ */
