/******************************************************************************
 * \file ble_pir.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-02
 *
 * \brief BLE PIR sensor slot table interface for ESP32 hub node.
 *
 * \details Dynamic slot table for PIR_* BLE devices.
 *          Mirrors the temperature sensor pattern in ble_temp.h.
 *
 *          Slot state machine:
 *          SLOT_EMPTY   — never seen or expired after PIR_REMOVE_MS
 *          SLOT_ACTIVE  — advertising within PIR_OFFLINE_MS
 *          SLOT_OFFLINE — last seen > PIR_OFFLINE_MS, tile stays visible
 *
 *          Occupancy logic is delegated to pir_window_update() on every
 *          received advertisement. Per-slot occupied state is read back
 *          via pir_window_get_occupied() in ble_manager.c.
 *
 * \note    PIR slot table (2026-05-20):
 *          handle_pir() replaced by dynamic slot table matching reed
 *          pattern. Extracted from ble_scan.c (2026-06-02).
 *
 * \note    Rename (2026-06-02):
 *          ble_pir_get_count() -> ble_pir_get_count()
 *          ble_pir_get_slot_info() -> ble_pir_get_slot_info()
 *          Wrappers in ble_manager.c removed; callers use these directly.
 ******************************************************************************/

#ifndef INCLUDE_BLE_PIR_H_
#define INCLUDE_BLE_PIR_H_

#include <stdbool.h>
#include <stdint.h>

/****************************** FUNCTION PROTOTYPES ***************************/

/******************************************************************************
 * \brief Pre-initialize the PIR sensor module.
 *
 * \return void
 *
 * \details Creates PIR mutex and zeroes the slot table.
 *          Must be called before ble_scan_start().
 ******************************************************************************/
void ble_pir_preinit(void);

/******************************************************************************
 * \brief Handle a PIR_* BLE advertisement.
 *
 * \param p_mfg   - Pointer to manufacturer data payload (after company ID).
 * \param mfg_len - Length of manufacturer data in bytes.
 * \param p_mac   - Pointer to 6-byte device MAC address.
 * \param p_name  - Null-terminated device name string.
 *
 * \return void
 *
 * \details Implements PIR slot state machine. Known MACs update their
 *          slot. New MACs are allocated a slot if table is not full.
 *          Delegates occupancy logic to pir_window_update().
 ******************************************************************************/
void ble_pir_handle(const uint8_t *p_mfg,
                    int            mfg_len,
                    const uint8_t *p_mac,
                    const char    *p_name);

/******************************************************************************
 * \brief Expire stale PIR sensor slots based on age thresholds.
 *
 * \return void
 *
 * \details Transitions ACTIVE->OFFLINE after PIR_OFFLINE_MS and
 *          OFFLINE->EMPTY after PIR_REMOVE_MS. Call periodically.
 ******************************************************************************/
void ble_expire_pir_slots(void);

/******************************************************************************
 * \brief Get count of active or offline PIR sensor slots.
 *
 * \return int - Highest non-empty slot index + 1, or 0 if none.
 ******************************************************************************/
int ble_pir_get_count(void);

/******************************************************************************
 * \brief Get slot metrics for an active PIR sensor.
 *
 * \param slot        - Slot index (0-based).
 * \param p_count_out - Output for cumulative motion count, or NULL.
 * \param p_batt_out  - Output for battery SOC percent, or NULL.
 * \param p_age_out   - Output for age in seconds, or NULL.
 *
 * \return bool - true if slot is active or offline, false if empty or OOB.
 ******************************************************************************/
bool ble_pir_get_slot_info(int       slot,
                            uint32_t *p_count_out,
                            int      *p_batt_out,
                            uint16_t *p_age_out);

#endif /* INCLUDE_BLE_PIR_H_ */
