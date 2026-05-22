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
 *
 * \note    Per-slot occupancy (2026-05-20):
 *          Both functions now take a slot index (0..MAX_PIRS-1).
 *          Each slot has its own independent circular buffer, hold timer,
 *          and occupied flag. The single-window globals are replaced by
 *          arrays indexed by slot. ble_get_pir_occupied() in ble_manager.c
 *          updated to take a slot argument. tcp_manager.c reads per-slot
 *          occupied into PIR_SLOT_STATE_T.occupied.
 ******************************************************************************/

#ifndef PIR_WINDOW_H
#define PIR_WINDOW_H

#include <stdint.h>
#include "config.h"

/******************************************************************************
 * \brief  Ingest one PIR advertisement sample and recompute occupancy.
 *
 * \param  slot    Slot index (0..MAX_PIRS-1). No-op if out of range.
 * \param  now_ms  Current FreeRTOS tick in milliseconds.
 * \param  occ     Raw occupied flag from BLE advertisement (0 or 1).
 *
 * \return void
 ******************************************************************************/
void pir_window_update(int slot, uint32_t now_ms, int occ);

/******************************************************************************
 * \brief  Return the current PIR occupancy state for one slot.
 *
 * \param  slot  Slot index (0..MAX_PIRS-1).
 *
 * \return int - 1 if occupied, 0 if empty or slot out of range.
 ******************************************************************************/
int pir_window_get_occupied(int slot);

#endif /* PIR_WINDOW_H */
