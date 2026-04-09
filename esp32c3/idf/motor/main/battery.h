/******************************************************************************
 * \file    battery.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Battery voltage measurement API for ESP32-C3 motor controller.
 ******************************************************************************/

#ifndef BATTERY_H
#define BATTERY_H

#include "esp_adc/adc_oneshot.h"

/**
 * \brief  Initialise battery ADC channel and enable GPIO.
 *
 * \note   Shares the ADC unit handle created in motor_control.c (g_adc1_handle).
 *         Call after adc_init() in app_main, passing g_adc1_handle.
 *
 * \param  adc_handle  Existing ADC_UNIT_1 handle from motor_control.c
 * \return 0 on success, -1 on error
 */
int battery_init(adc_oneshot_unit_handle_t adc_handle);

/**
 * \brief  Read battery voltage in millivolts.
 *
 *         Enables divider via GPIO5, waits 10ms for ADC to settle,
 *         reads ADC1_CH1 (GPIO1), disables divider, reconstructs Vbat.
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
