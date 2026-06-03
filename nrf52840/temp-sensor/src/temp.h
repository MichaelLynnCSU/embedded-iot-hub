/******************************************************************************
 * \file    temp.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    2026-06-02
 *
 * \brief   TMP36 temperature sensor interface for nRF52840 temp sensor node.
 *
 * \details Single TMP36 read via SAADC channel 1 (AIN1, P0.03).
 *          Returns temperature in tenths of °C (int16_t) so the full
 *          signed range fits in the two-byte BLE payload without floats.
 *          Returns TEMP_READ_ERROR on ADC failure.
 *
 *          Hardware:
 *          - Board:    Teyleten Robot Pro Micro nRF52840
 *          - TMP36:    VCC → 2.5V VDD rail (HX3001 LDO output)
 *                      GND → GND
 *                      OUT → P0.03 (AIN1) — SAADC channel 1
 *
 * \warning VDD on this board is 2.5V (HX3001 LDO). TMP36 lower operating
 *          limit rises to approximately -25°C at 2.5V supply. Acceptable
 *          for indoor/ambient monitoring. For sub-zero operation power the
 *          TMP36 from an external 3V rail.
 *
 * \note    TMP36 transfer function:
 *          Vout (mV) = 500 + (temp_C * 10)
 *          temp_C    = (Vout_mV - 500) / 10
 *          Returned as tenths of °C: decidegC = Vout_mV - 500
 *          e.g. 25.0°C → 750 mV → returns 250
 *               -10.0°C → 400 mV → returns -100
 ******************************************************************************/

#ifndef TEMP_H
#define TEMP_H

#include <stdint.h>

/** Sentinel returned on ADC read failure. */
#define TEMP_READ_ERROR  INT16_MIN

/**
 * \brief  Initialise SAADC channel 1 for TMP36 measurement.
 * \return 0 on success, negative errno on failure.
 */
int temp_init(void);

/**
 * \brief  Read temperature from TMP36 on AIN1 (P0.03).
 * \return Temperature in tenths of °C, or TEMP_READ_ERROR on failure.
 *         Divide by 10 for degrees C.  e.g. 253 → 25.3 °C.
 */
int16_t temp_read_decidegc(void);

/**
 * \brief  Log current temperature reading at INFO level.
 */
void temp_print_status(void);

#endif /* TEMP_H */
