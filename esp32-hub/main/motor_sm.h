/******************************************************************************
 * \file    motor_sm.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Motor state machine and health ping interface for ESP32 hub.
 *
 * \details Exposes the three-state motor lifecycle (IDLE / RUNNING /
 *          COOLDOWN), the health ping that keeps the BeagleBone timeout
 *          satisfied during idle periods, and a clean result type so
 *          tcp_manager.c can apply the ping outcome without knowing
 *          motor_sm internals.
 *
 * \note    All internal state (MOTOR_SM_T, g_motor_sm, g_last_ping_ms)
 *          is private to motor_sm.c. tcp_manager.c queries state through
 *          motor_sm_get_state() and motor_sm_ping_due().
 ******************************************************************************/

#ifndef MOTOR_SM_H
#define MOTOR_SM_H

#include <stdint.h>
#include <stdbool.h>
#include "lwip/sockets.h"

/* Motor state machine states ------------------------------------------------*/
#define MOTOR_IDLE       0   /**< motor off, C3 disconnected */
#define MOTOR_RUNNING    1   /**< motor on, enforcing MIN_RUN_MS */
#define MOTOR_COOLDOWN   2   /**< PWM=0 sent, waiting cooldown */

/* Timing constants ----------------------------------------------------------*/
/** 10-minute minimum run -- protects motor from rapid stop/start cycling. */
#define MIN_RUN_MS             (10UL * 60UL * 1000UL)

/** 5-minute post-stop cooldown before motor is allowed to start again. */
#define COOLDOWN_MS            (5UL  * 60UL * 1000UL)

/** Idle health ping interval. Must be < BeagleBone motor timeout (300 s).
 *  240 s gives a 60 s margin. */
#define MOTOR_PING_INTERVAL_MS (240UL * 1000UL)

/** Per-attempt connect + recv deadline for the health ping. */
#define MOTOR_PING_TIMEOUT_MS  3000

/** Receive buffer size for ping response. */
#define MOTOR_PING_RX_BUF      64

/** \brief Result returned by ping_motor_for_health(). */
typedef struct
{
   bool success;   /*!< true if {"batt_motor":<pct>} received successfully */
   int  batt_pct;  /*!< battery SOC percent, valid only when success=true */
} MOTOR_PING_RESULT_T;

/******************************************************************************
 * \brief  Return the current motor state machine state.
 *
 * \return One of MOTOR_IDLE, MOTOR_RUNNING, MOTOR_COOLDOWN.
 ******************************************************************************/
uint8_t motor_sm_get_state(void);

/******************************************************************************
 * \brief  Return true if the idle health ping is due this cycle.
 *
 * \param  now_ms  Current FreeRTOS tick in milliseconds.
 *
 * \return true if (now_ms - last_ping_ms) >= MOTOR_PING_INTERVAL_MS.
 *
 * \details Initialised to fire immediately on boot (last_ping_ms = 0) so
 *          the first reading is available before MOTOR_PING_INTERVAL_MS
 *          elapses.
 ******************************************************************************/
bool motor_sm_ping_due(uint32_t now_ms);

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
float run_motor_sm(float      pi_out_pct,
                   uint32_t   now_ms,
                   bool      *p_connect,
                   bool      *p_disconnect);

/******************************************************************************
 * \brief  Open a short-lived TCP connection to the C3, send {"ping":1},
 *         read {"batt_motor":<pct>} back, and close.
 *
 * \param  p_addr   Pre-filled sockaddr_in for the C3 motor.
 * \param  now_ms   Current FreeRTOS tick in milliseconds (used to update
 *                  the last-ping timestamp so ping_due resets correctly).
 *
 * \return MOTOR_PING_RESULT_T -- caller applies outcome to g_state.
 *
 * \details Called only when motor SM is MOTOR_IDLE so it cannot collide
 *          with an active run connection. Consumes the "READY" banner sent
 *          by tcp_server before issuing the ping command. Uses a
 *          select()-based MOTOR_PING_TIMEOUT_MS deadline for both connect
 *          and recv phases.
 ******************************************************************************/
MOTOR_PING_RESULT_T ping_motor_for_health(struct sockaddr_in *p_addr,
                                           uint32_t            now_ms);

#endif /* MOTOR_SM_H */
