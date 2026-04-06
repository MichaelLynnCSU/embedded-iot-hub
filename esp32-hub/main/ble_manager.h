/******************************************************************************
 * \file ble_manager.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief BLE manager public interface for ESP32 hub node.
 *
 * \details Exposes BLE stack lifecycle, device state accessors, command
 *          API, and dynamic reed sensor discovery. See ble_manager.c,
 *          ble_scan.c, ble_lock.c, and ble_light.c for implementations.
 ******************************************************************************/

#ifndef INCLUDE_BLE_MANAGER_H_
#define INCLUDE_BLE_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>
#include "freertos/event_groups.h"

/******************************** CONSTANTS ***********************************/

#define BLE_INITIALIZED_BIT  BIT0  /**< set in ble_event_group when scan active */
#define BLE_DEV_PIR          0     /**< PIR motion sensor device index */
#define BLE_DEV_REED1        1     /**< reed sensor 1 device index */
#define BLE_DEV_REED2        2     /**< reed sensor 2 device index */
#define BLE_DEV_LIGHT        3     /**< smart light device index */
#define BLE_DEV_LOCK         4     /**< smart lock device index */

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

/** \brief Get current PIR motion count.
 *  \return int - Motion event count. */
int ble_get_motion_count(void);

/** \brief Get current light relay state.
 *  \return uint8_t - Light state. */
uint8_t ble_get_light_state(void);

/** \brief Get current lock state.
 *  \return uint8_t - Lock state. */
uint8_t ble_get_lock_state(void);

/** \brief Get PIR sensor battery SOC.
 *  \return int - Battery SOC percent. */
int ble_get_pir_batt(void);

/** \brief Get smart lock battery SOC.
 *  \return int - Battery SOC percent. */
int ble_get_lock_batt(void);

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

/** \brief Pre-initialize the BLE scan module.
 *  \return void */
void ble_scan_preinit(void);

/** \brief Start BLE scanning.
 *  \return void */
void ble_scan_start(void);

/** \brief Get count of active reed sensor slots.
 *  \return int - Number of active reed sensors. */
int ble_get_reed_count(void);

/** \brief Get info for a reed sensor slot.
 *  \param slot       - Slot index (0-based).
 *  \param p_name_out - Output buffer for device name (31 chars max), or NULL.
 *  \param p_batt_out - Output for battery SOC, or NULL.
 *  \param p_age_out  - Output for age in seconds, or NULL.
 *  \param p_state_out - Output for door state, or NULL.
 *  \param p_gen_out  - Output for generation counter, or NULL.
 *  \return bool - true if slot is active, false if empty or out of range. */
bool ble_get_reed_slot_info(int slot,
                             char     *p_name_out,
                             int      *p_batt_out,
                             uint16_t *p_age_out,
                             uint8_t  *p_state_out,
                             uint16_t *p_gen_out);

/** \brief Expire stale reed sensor slots.
 *  \return void */
void ble_expire_reed_slots(void);

#endif /* INCLUDE_BLE_MANAGER_H_ */
