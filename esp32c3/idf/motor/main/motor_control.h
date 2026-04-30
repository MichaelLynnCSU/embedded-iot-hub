/******************************************************************************
 * \file    motor_control.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Motor control task and JSON parser for ESP32-C3 motor node.
 ******************************************************************************/

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "esp_adc/adc_oneshot.h"

/**
 * \brief  Configure motor direction GPIO pins (IN1, IN2) as outputs and
 *         drive both low. Must be called before motor_enable() or motor_task().
 *
 * \return 0 on success, -1 on failure.
 */
int motor_init(void);

/**
 * \brief  Enable motor direction pins (IN1=1, IN2=0).
 *         Called by tcp_server on first data received from hub.
 */
void motor_enable(void);

/**
 * \brief  Parse a JSON object received over TCP and update motor state.
 *         Handles keys: "low", "high", "motor", "avg_temp".
 *
 * \param  p_buf  Null-terminated JSON string
 */
void parse_tcp_json(const char *p_buf);

/**
 * \brief  Motor control task. Drives PWM from temperature or knob ADC.
 *         Reads battery every BATT_INTERVAL_MS and sends SOC percent to hub.
 *         Never returns -- run via xTaskCreate().
 */
void motor_task(void *p_arg);

/**
 * \brief  Initialise PWM timer and channel.
 */
void pwm_init(void);

/**
 * \brief  Initialise ADC unit and knob channel (ADC1_CH0 / GPIO0).
 *         Returns handle for sharing with battery_init().
 */
void adc_init(adc_oneshot_unit_handle_t *p_handle);

#endif /* MOTOR_CONTROL_H */
