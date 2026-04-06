/******************************************************************************
 * \file battery.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Battery voltage measurement interface for nRF52840 reed sensor.
 *
 * \details Provides ADC-based battery voltage reading via a resistor
 *          divider. See battery.c for implementation details.
 ******************************************************************************/

#ifndef INCLUDE_BATTERY_H_
#define INCLUDE_BATTERY_H_

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Initialize battery ADC channel and divider enable GPIO.
 *  \return int - 0 on success, negative errno on failure. */
int battery_init(void);

/** \brief Read battery voltage in millivolts.
 *  \return int - Battery voltage in mV on success, -EIO on failure. */
int battery_read_mv(void);

/** \brief Print battery voltage and status to console and flash log.
 *  \return void */
void battery_print_status(void);

#endif /* INCLUDE_BATTERY_H_ */
