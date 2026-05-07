/******************************************************************************
 * \file    pir_window.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   PIR occupancy sliding-window interface for BLE scan module.
 *
 * \details Exposes pir_window_update() and pir_window_get_occupied().
 *          All state (circular buffer, hold timer) is private to
 *          pir_window.c. Window width, threshold, and hold duration are
 *          driven by config.h constants PIR_WINDOW_SEC,
 *          PIR_WINDOW_THRESHOLD, and PIR_HOLD_SEC.
 ******************************************************************************/

#ifndef PIR_WINDOW_H
#define PIR_WINDOW_H

#include <stdint.h>

/******************************************************************************
 * \brief  Ingest one PIR advertisement sample and recompute occupancy.
 *
 * \param  now_ms  Current FreeRTOS tick in milliseconds.
 * \param  occ     Raw occupied flag from BLE advertisement (0 or 1).
 *
 * \return void
 *
 * \details On occ=1 the timestamp is written into a circular buffer.
 *          The window counts buffered events within PIR_WINDOW_SEC.
 *          When the count reaches PIR_WINDOW_THRESHOLD the hold timer
 *          is (re)started. Occupancy remains 1 for PIR_HOLD_SEC after
 *          the last threshold crossing and drops to 0 once the hold
 *          expires.
 ******************************************************************************/
void pir_window_update(uint32_t now_ms, int occ);

/******************************************************************************
 * \brief  Return the current PIR occupancy state.
 *
 * \return int - 1 if occupied, 0 if empty.
 ******************************************************************************/
int pir_window_get_occupied(void);

#endif /* PIR_WINDOW_H */
