/******************************************************************************
 * \file uart_staging.h
 * \brief UART protocol transitions — partial state updaters and reed logic.
 *
 * \details Owns reed transition detection and UART-specific partial writes
 *          into the canonical state registry. Does not own any state itself.
 ******************************************************************************/

#ifndef INCLUDE_UART_STAGING_H_
#define INCLUDE_UART_STAGING_H_

#include "../controller_internal.h"

void uart_stage_lock  (int lock_state, int batt);
void uart_stage_pir   (int val);
void uart_stage_light (int val);
void uart_check_reeds (const struct SensorData *p_data);

#endif /* INCLUDE_UART_STAGING_H_ */
