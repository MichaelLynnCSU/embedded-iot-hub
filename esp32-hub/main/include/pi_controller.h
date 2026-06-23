/******************************************************************************
 * \file    pi_controller.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   PI controller interface for ESP32 hub TCP manager.
 *
 * \details Exposes run_pi_controller()
 *          used by tcp_manager.c when scaling percent to duty counts.
 *          All state (PI_STATE_T) is private to pi_controller.c.
 ******************************************************************************/

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <stdint.h>


/******************************************************************************
 * \brief  Run one PI controller iteration.
 *
 * \param  now_ms  Current FreeRTOS tick in milliseconds.
 *
 * \return Effective PWM output [0.0, 100.0] percent.
 *
 * \details Reads setpoint and gains from device_gateway getters. Accumulates
 *          integral with anti-windup clamp. Safe to call every TCP loop
 *          iteration -- dt is derived from elapsed time since last call.
 ******************************************************************************/
float run_pi_controller(uint32_t now_ms);

#endif /* PI_CONTROLLER_H */
