/******************************************************************************
 * \file    main.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Top-level constants for ESP32-C3 motor controller node.
 *
 * \details Timing, TCP protocol, and PWM constants shared across
 *          motor_control.c and the unit test suite. Centralised here
 *          so tests can assert on protocol boundaries without pulling in
 *          FreeRTOS or ESP-IDF headers.
 *
 *          Battery constants live in battery.h -- not here. The ADC
 *          calibration and threshold values are hardware-specific to the
 *          battery driver and are tested via battery.h directly.
 *
 * \note    Power saving (2026-05-04):
 *          MOTOR_LOOP_MS raised from 100ms to 5000ms. Previously the motor
 *          task looped at 100ms applying PWM and reading ADC continuously.
 *          With knob ADC removed and PID logic moved to the hub, 5s resolution
 *          is sufficient -- hub sends PWM updates on its own TCP_SEND_INTERVAL.
 *          Combined with light sleep when idle, average current drops
 *          significantly from always-on WiFi baseline (~81mA).
 *
 *          MIN_RUN_SEC=600 enforced by hub state machine -- motor must run
 *          for at least 10 minutes once started to protect against rapid
 *          stop/start cycling near the temperature threshold.
 ******************************************************************************/

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

/* ---- Timing ---- */
#define MOTOR_LOOP_MS        5000u           /**< motor task loop period — raised from 100ms for power saving */
#define STATS_INTERVAL_MS    60000u          /**< heap/task stats log interval  */
#define BATT_INTERVAL_MS     30000u          /**< battery read and report interval */
#define RECV_TIMEOUT_MS      30000u          /**< TCP client idle timeout       */
#define ACCEPT_TIMEOUT_SEC   2               /**< accept() WDT kick interval    */
#define MIN_RUN_SEC          600             /**< minimum motor run time seconds — hub enforced */

/* ---- TCP protocol ---- */
#define TCP_PORT             3333
#define BATT_JSON_BUF_SIZE   48              /**< {"batt_motor":100} + null     */
#define BATT_SAG_REJECT_MV   200             /**< WiFi TX burst sag reject threshold */

/* ---- PWM ---- */
#define PWM_DUTY_RES         13
#define PWM_DUTY_MAX         ((1 << PWM_DUTY_RES) - 1)

#endif /* MAIN_H */
