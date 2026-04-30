/******************************************************************************
 * \file    main.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Top-level constants for ESP32-C3 motor controller node.
 *
 * \details Timing, TCP protocol, PWM, and motor control constants shared
 *          across motor_control.c and the unit test suite. Centralised here
 *          so tests can assert on protocol boundaries without pulling in
 *          FreeRTOS or ESP-IDF headers.
 *
 *          Battery constants live in battery.h -- not here. The ADC
 *          calibration and threshold values are hardware-specific to the
 *          battery driver and are tested via battery.h directly, matching
 *          the nRF52840 smart-lock pattern where battery.h owns VBAT_*
 *          and mv_to_soc() and main.h owns only timing and protocol.
 ******************************************************************************/

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

/* ---- Timing ---- */
#define MOTOR_LOOP_MS        100u            /**< motor task loop period        */
#define STATS_INTERVAL_MS    60000u          /**< heap/task stats log interval  */
#define BATT_INTERVAL_MS     30000u          /**< battery read and report       */
#define RECV_TIMEOUT_MS      30000u          /**< TCP client idle timeout       */
#define ACCEPT_TIMEOUT_SEC   2               /**< accept() WDT kick interval    */

/* ---- TCP protocol ---- */
#define TCP_PORT             3333
#define BATT_JSON_BUF_SIZE   48              /**< {"batt_motor":100} + null     */
#define BATT_SAG_REJECT_MV   200             /**< WiFi TX burst sag reject      */

/* ---- PWM ---- */
#define PWM_DUTY_RES         13
#define PWM_DUTY_MAX         ((1 << PWM_DUTY_RES) - 1)

/* ---- Motor control defaults ---- */
#define ADC_MAX_RAW          4095
#define DEFAULT_AWS_LOW      20
#define DEFAULT_AWS_HIGH     35
#define DEFAULT_AVG_TEMP     25
#define LOG_THROTTLE_N       20u

/* ---- PWM duty from temperature ---- */
static inline uint32_t motor_temp_to_duty(int t, int low, int high)
{
    if (t < low)        { t = low;  }
    if (t > high)       { t = high; }
    if (high <= low)    { return 0; }
    return (uint32_t)PWM_DUTY_MAX *
           (uint32_t)(t - low) /
           (uint32_t)(high - low);
}

#endif /* MAIN_H */
