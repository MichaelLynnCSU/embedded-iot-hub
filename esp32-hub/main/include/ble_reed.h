/******************************************************************************
 * \file ble_reed.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-02
 *
 * \brief BLE reed sensor slot table interface for ESP32 hub node.
 *
 * \details Dynamic slot table for ReedSensor* BLE devices.
 *          Mirrors the temperature sensor pattern in ble_temp.h.
 *
 *          Slot state machine:
 *          SLOT_EMPTY   — never seen or expired after REED_REMOVE_MS
 *          SLOT_ACTIVE  — advertising within REED_OFFLINE_MS
 *          SLOT_OFFLINE — last seen > REED_OFFLINE_MS, tile stays visible
 *
 *          Thresholds (from config.h):
 *          - REED_OFFLINE_MS : 150 s — matches BLE_AGE_THRESHOLD_S on STM32
 *          - REED_REMOVE_MS  : 3600 s — 1 hour unseen, slot cleared
 *
 *          Cooldown window prevents immediate slot re-allocation after
 *          a device is removed. Duration driven by BLE_COOLDOWN_MS.
 ******************************************************************************/

#ifndef INCLUDE_BLE_REED_H_
#define INCLUDE_BLE_REED_H_

#include <stdbool.h>
#include <stdint.h>

/****************************** FUNCTION PROTOTYPES ***************************/

/******************************************************************************
 * \brief Pre-initialize the reed sensor module.
 *
 * \return void
 *
 * \details Creates reed mutex and zeroes slot and cooldown tables.
 *          Must be called before ble_scan_start().
 ******************************************************************************/
void ble_reed_preinit(void);

/******************************************************************************
 * \brief Handle a ReedSensor* BLE advertisement.
 *
 * \param p_mfg   - Pointer to manufacturer data payload (after company ID).
 * \param mfg_len - Length of manufacturer data in bytes.
 * \param p_mac   - Pointer to 6-byte device MAC address.
 * \param p_name  - Null-terminated device name string.
 *
 * \return void
 *
 * \details Implements reed slot state machine. Known MACs update their
 *          slot. New MACs are allocated a slot if not in cooldown and
 *          table is not full.
 ******************************************************************************/
void ble_reed_handle(const uint8_t *p_mfg,
                     int            mfg_len,
                     const uint8_t *p_mac,
                     const char    *p_name);

/******************************************************************************
 * \brief Expire stale reed sensor slots based on age thresholds.
 *
 * \return void
 *
 * \details Transitions ACTIVE->OFFLINE after REED_OFFLINE_MS and
 *          OFFLINE->EMPTY after REED_REMOVE_MS. Call periodically.
 ******************************************************************************/
void ble_expire_reed_slots(void);

/******************************************************************************
 * \brief Get count of active or offline reed sensor slots.
 *
 * \return int - Highest non-empty slot index + 1, or 0 if none.
 ******************************************************************************/
int ble_get_reed_count(void);

/******************************************************************************
 * \brief Get information for a reed sensor slot.
 *
 * \param slot        - Slot index (0-based).
 * \param p_name_out  - Output buffer for device name (31 chars max), or NULL.
 * \param p_batt_out  - Output for battery SOC percent, or NULL.
 * \param p_age_out   - Output for age in seconds, or NULL.
 * \param p_state_out - Output for door state byte, or NULL.
 * \param p_gen_out   - Output for generation counter, or NULL.
 *
 * \return bool - true if slot is active or offline, false if empty or OOB.
 ******************************************************************************/
bool ble_get_reed_slot_info(int       slot,
                             char     *p_name_out,
                             int      *p_batt_out,
                             uint16_t *p_age_out,
                             uint8_t  *p_state_out,
                             uint16_t *p_gen_out);

#endif /* INCLUDE_BLE_REED_H_ */
