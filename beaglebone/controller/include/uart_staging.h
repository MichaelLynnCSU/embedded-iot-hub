/******************************************************************************
 * \file uart_staging.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-09
 *
 * \brief UART protocol transition logic for BeagleBone data controller.
 *
 * \details Owns reed slot generation detection and online/offline transition
 *          logging. Provides partial-update wrappers for UART-ingress frames
 *          that delegate all state writes to state_registry internally.
 *          Does not own any state.
 ******************************************************************************/
#ifndef INCLUDE_CMD_UART_STAGING_H_
#define INCLUDE_CMD_UART_STAGING_H_

#include "sensor_types.h"

void uart_stage_lock  (int lock_state, int batt);
void uart_stage_pir   (int val);
void uart_stage_light (int val);
void uart_check_reeds (const struct SensorData *p_data);

#endif
