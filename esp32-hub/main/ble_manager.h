/******************************************************************************
 * \file ble_manager.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief BLE manager public interface for ESP32 hub node.
 *
 * \details Exposes BLE stack lifecycle, device state accessors, and command
 *          API. Reed and PIR slot APIs are exposed directly via ble_reed.h
 *          and ble_pir.h — callers include those headers directly.
 *          See ble_manager.c, ble_scan.c, ble_lock.c, and ble_light.c
 *          for implementations.
 *
 * \note    PIR slot table (2026-05-20):
 *          ble_get_motion_count(), ble_get_pir_batt(), and the single-device
 *          scan accessors removed. Replaced with ble_get_pir_slot_info() and
 *          ble_get_pir_count() mirroring the reed slot API. BLE_DEV_PIR2 and
 *          updated BLE_DEV_REED1/2/LIGHT/LOCK indices added to match the
 *          expanded BLE_DEV_IDX_E enum in ble_internal.h.
 *
 * \note    Wrapper removal (2026-06-02):
 *          ble_get_pir_slot_info(), ble_get_pir_count(), ble_get_reed_count(),
 *          ble_get_reed_slot_info(), and ble_expire_reed_slots() removed.
 *          Callers include ble_pir.h and ble_reed.h directly.
 ******************************************************************************/

#ifndef INCLUDE_BLE_MANAGER_H_
#define INCLUDE_BLE_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>
#include "freertos/event_groups.h"

/******************************** CONSTANTS ***********************************/

#define BLE_INITIALIZED_BIT  BIT0  /**< set in ble_event_group when scan active */

/** \brief BLE device indices — must match BLE_DEV_IDX_E in ble_internal.h */
#define BLE_DEV_PIR          0     /**< PIR slot 0 — PIR_Motion  */
#define BLE_DEV_PIR2         1     /**< PIR slot 1 — PIR_Motion2 */
#define BLE_DEV_REED1        2     /**< reed sensor 1             */
#define BLE_DEV_REED2        3     /**< reed sensor 2             */
#define BLE_DEV_LIGHT        4     /**< smart light relay         */
#define BLE_DEV_LOCK         5     /**< smart lock                */

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Initialize the BLE manager.
 *  \return void */
void ble_manager_init(void);

/** \brief BLE manager FreeRTOS task.
 *  \param p_system_eg - System event group handle.
 *  \param p_ble_eg    - BLE event group, sets BLE_INITIALIZED_BIT on ready.
 *  \return void */
void ble_manager_task(EventGroupHandle_t p_system_eg,
                      EventGroupHandle_t p_ble_eg);

/** \brief Get seconds since a device was last seen.
 *  \param idx - Device index (BLE_DEV_* constants).
 *  \return uint16_t - Age in seconds, 0xFFFF if never seen. */
uint16_t ble_get_device_age_s(int idx);

/** \brief Get current light relay state.
 *  \return uint8_t - Light state. */
uint8_t ble_get_light_state(void);

/** \brief Get current lock state.
 *  \return uint8_t - Lock state. */
uint8_t ble_get_lock_state(void);

/** \brief Get smart lock battery SOC.
 *  \return int - Battery SOC percent. */
int ble_get_lock_batt(void);

/** \brief Get PIR occupancy from shared sliding window.
 *  \param slot - Zero-based PIR slot index (0..MAX_PIRS-1).
 *  \return int - 1 if occupied, 0 if empty or out of range. */
int ble_get_pir_occupied(int slot);

/** \brief Send relay state command to light node.
 *  \param state - Relay state (0=off, 1=on).
 *  \return void */
void ble_send_light_command(uint8_t state);

/** \brief Send lock state command to lock node.
 *  \param state - Lock state (0=unlock, 1=lock).
 *  \return void */
void ble_send_lock_command(uint8_t state);

/** \brief Update room sensor state by sensor ID.
 *  \param sensor_id - Sensor ID to match in rooms array.
 *  \param p_state   - Null-terminated state string to set.
 *  \return void */
void ble_update_room_sensor(int sensor_id, const char *p_state);

#endif /* INCLUDE_BLE_MANAGER_H_ */
