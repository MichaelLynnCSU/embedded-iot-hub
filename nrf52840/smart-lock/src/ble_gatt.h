/******************************************************************************
 * \file    ble_gatt.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   BLE GATT service API for nRF52840 smart lock node.
 ******************************************************************************/

#ifndef BLE_GATT_H
#define BLE_GATT_H

#include <stdint.h>
#include "config.h"   /* STATS_INTERVAL_SEC, IDLE_HEARTBEAT_SEC, BATT_UPDATE_SEC,
                         MFG_*, LOCK_WRITE_LEN */

/**
 * \brief Initialise BLE stack, GATT service, and start advertising.
 * \return 0 on success, negative errno on failure.
 */
int ble_gatt_init(void);

/**
 * \brief Update BLE advertisement manufacturer data with current lock
 *        state and battery SOC.
 * \return void
 */
void ble_adv_update(void);

/**
 * \brief Update battery SOC and notify connected client.
 * \param soc - Battery SOC percent (0-100).
 * \return void
 */
void ble_set_batt(uint8_t soc);

/**
 * \brief Set lock state directly (e.g. on boot from settings restore).
 * \param state - Raw lock state value.
 * \return void
 */
void ble_set_lock_state(uint8_t state);

/**
 * \brief Called by motor.c when motor_off_timer fires.
 *        Advances lock state machine UNLOCKING->UNLOCKED or LOCKING->LOCKED.
 *        Saves settled state to settings, notifies BLE client, updates adv.
 * \return void
 */
void ble_lock_settle(void);

#endif /* BLE_GATT_H */
