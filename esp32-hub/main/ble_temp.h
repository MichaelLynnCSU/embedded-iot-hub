/******************************************************************************
 * \file ble_temp.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-02
 *
 * \brief BLE temperature sensor handler public interface for ESP32 hub node.
 *
 * \details Dynamic slot table for TempSensor* BLE devices.
 *          Mirrors the reed sensor pattern in ble_scan.c / ble_manager.h.
 *
 *          MFG data layout (4 bytes, company ID 0xAE):
 *          [0] = 0xAE  (company ID — already consumed by find_mfg_data)
 *          [1] = temp_decidegc low byte   (int16 little-endian)
 *          [2] = temp_decidegc high byte
 *          [3] = batt_soc                 (0-100%)
 ******************************************************************************/

#ifndef INCLUDE_BLE_TEMP_H_
#define INCLUDE_BLE_TEMP_H_

#include <stdbool.h>
#include <stdint.h>

/** \brief Pre-initialize the temp slot table and mutex.
 *         Call from ble_scan_preinit() before scanning starts.
 *  \return void */
void ble_temp_preinit(void);

/** \brief Handle a TempSensor* advertisement.
 *  \param p_mfg   - Manufacturer data payload pointer.
 *  \param mfg_len - Manufacturer data length in bytes.
 *  \param p_mac   - Pointer to 6-byte device MAC address.
 *  \param p_name  - Null-terminated device name string.
 *  \return void */
void ble_temp_handle(const uint8_t *p_mfg,
                     int            mfg_len,
                     const uint8_t *p_mac,
                     const char    *p_name);

/** \brief Expire stale temperature sensor slots.
 *         Call periodically alongside ble_expire_reed_slots().
 *  \return void */
void ble_expire_temp_slots(void);

/** \brief Get count of active or offline temperature sensor slots.
 *  \return int - Highest non-empty slot index + 1, or 0 if none. */
int ble_get_temp_count(void);

/** \brief Get information for a temperature sensor slot.
 *  \param slot       - Slot index (0-based).
 *  \param p_name_out - Output buffer for device name (32 chars max), or NULL.
 *  \param p_temp_out - Output for temperature in tenths of °C, or NULL.
 *  \param p_batt_out - Output for battery SOC percent, or NULL.
 *  \param p_age_out  - Output for age in seconds, or NULL.
 *  \param p_gen_out  - Output for generation counter, or NULL.
 *  \return bool - true if slot is active or offline, false if empty or OOB. */
bool ble_get_temp_slot_info(int      slot,
                             char    *p_name_out,
                             int16_t *p_temp_out,
                             int     *p_batt_out,
                             uint16_t *p_age_out,
                             uint16_t *p_gen_out);

#endif /* INCLUDE_BLE_TEMP_H_ */
