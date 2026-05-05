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
 *          Node enters light sleep when PWM=0 and no client is connected.
 *          On-demand TCP from the hub eliminates always-on WiFi TX bursts
 *          which were the dominant current draw. Knob ADC and temp logic
 *          removed entirely -- hub owns all control decisions.
 ******************************************************************************/

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

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
 * \brief  Parse a JSON object received over TCP and update PWM duty.
 *         Handles key: "pwm" (0-PWM_DUTY_MAX). All other keys ignored.
 *         Hub owns temp sensing and PID -- motor applies duty only.
 *
 * \param  p_buf  Null-terminated JSON string.
 */
void parse_tcp_json(const char *p_buf);

/**
 * \brief  Motor control task. Applies g_pwm_duty to LEDC each loop.
 *         Reads battery every BATT_INTERVAL_MS and sends SOC percent to hub.
 *         Enters light sleep when PWM=0 and no client connected.
 *         Never returns -- run via xTaskCreate().
 */
void motor_task(void *p_arg);

/**
 * \brief  Initialise PWM timer and channel.
 */
void pwm_init(void);

#endif /* MOTOR_CONTROL_H */
