/******************************************************************************
 * \file    motor_sm.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Motor state machine interface for ESP32 hub.
 *
 * \details Exposes the three-state motor lifecycle (IDLE / RUNNING /
 *          COOLDOWN). All internal state (MOTOR_SM_T, g_motor_sm) is
 *          private to motor_sm.c. tcp_manager.c drives the SM via
 *          run_motor_sm() each control loop iteration.
 *
 * \note    Ping removed (2026-05-XX):
 *          ping_motor_for_health(), motor_sm_ping_due(), motor_sm_get_state()
 *          and associated constants removed. Motor node now initiates contact
 *          on every deep-sleep wake so the hub always receives a fresh batt
 *          report without needing to probe.
 ******************************************************************************/

#ifndef MOTOR_SM_H
#define MOTOR_SM_H

#include <stdint.h>
#include <stdbool.h>

/* Motor state machine states ------------------------------------------------*/
#define MOTOR_IDLE       0   /**< motor off, C3 disconnected */
#define MOTOR_RUNNING    1   /**< motor on, enforcing MIN_RUN_MS */
#define MOTOR_COOLDOWN   2   /**< PWM=0 sent, waiting cooldown */

/* Timing constants ----------------------------------------------------------*/
/** 10-minute minimum run -- protects motor from rapid stop/start cycling. */
#define MIN_RUN_MS   (10UL * 60UL * 1000UL)

/** 5-minute post-stop cooldown before motor is allowed to start again. */
#define COOLDOWN_MS  (5UL  * 60UL * 1000UL)

/******************************************************************************
 * \brief  Run one motor state machine iteration.
 *
 * \param  pi_out_pct   Raw PI output [0.0, 100.0].
 * \param  now_ms       Current FreeRTOS tick in milliseconds.
 * \param  p_connect    Set true when C3 TCP connection should be opened.
 * \param  p_disconnect Set true when C3 TCP connection should be closed.
 *
 * \return Effective PWM percent [0.0, 100.0] to send to C3 this cycle.
 *         May differ from pi_out_pct during min-run enforcement or cooldown.
 ******************************************************************************/
/** Test seam: returns current SM state. No production caller. */
uint8_t motor_sm_get_state(void);

float run_motor_sm(float      pi_out_pct,
                   uint32_t   now_ms,
                   bool      *p_connect,
                   bool      *p_disconnect);

#endif /* MOTOR_SM_H */
