/******************************************************************************
 * \file    motor_control.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Motor control task and JSON parser for ESP32-C3 motor node.
 *
 * \details Motor is a pure PWM receiver. All temperature sensing, PID
 *          control, and duty calculation run on the ESP32 hub. This node
 *          receives {"pwm": X} over TCP and applies the duty directly.
 *          Battery SOC is read locally and sent back to the hub.
 *
 *          Power saving (2026-05-04):
 *          Node enters deep sleep between wake cycles. tcp_client_exchange()
 *          called once per wake; motor deep sleeps after applying duty.
 *          Knob ADC and temp logic removed -- hub owns all control decisions.
 *
 *          Deep sleep (2026-05-XX):
 *          get_pwm_duty() added so tcp_client.c can read the duty written by
 *          parse_tcp_json() without accessing g_pwm_duty directly.
 ******************************************************************************/
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

/**
 * \brief  Configure motor direction GPIO pins (IN1, IN2) as outputs and
 *         drive both low. Must be called before motor_enable() or motor_task().
 *
 * \return 0 on success, -1 on failure.
 */
int motor_init(void);

/**
 * \brief  Enable motor direction pins (IN1=1, IN2=0).
 *         Called by tcp_client after a real PWM command is parsed.
 */
void motor_enable(void);

/**
 * \brief  Parse a JSON object received over TCP and update PWM duty.
 *         Handles key: "pwm" (0-PWM_DUTY_MAX). All other keys ignored.
 *
 * \param  p_buf  Null-terminated JSON string.
 */
void parse_tcp_json(const char *p_buf);

/**
 * \brief  Return the last duty value written by parse_tcp_json().
 *         Used by tcp_client.c to decide whether to call motor_enable().
 *
 * \return Current PWM duty (0 -- PWM_DUTY_MAX).
 */
uint32_t get_pwm_duty(void);

/**
 * \brief  Motor control task. Calls tcp_client_exchange() once, applies duty,
 *         then enters deep sleep. Never returns -- run via xTaskCreate().
 */
void motor_task(void *p_arg);

/**
 * \brief  Initialise PWM timer and channel.
 */
void pwm_init(void);

#endif /* MOTOR_CONTROL_H */
