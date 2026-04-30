/******************************************************************************
 * \file    ble_gatt.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   BLE GATT service and advertising API for nRF52840 smart light.
 ******************************************************************************/

#ifndef BLE_GATT_H
#define BLE_GATT_H

#include <stdint.h>

/**
 * \brief  Initialise BLE stack, load settings, register GATT service,
 *         start connectable advertisement, arm heartbeat and stats timers.
 *
 * \return 0 on success, negative errno on failure.
 */
int ble_gatt_init(void);

/**
 * \brief  Update manufacturer data payload with current light state and
 *         refresh advertisement atomically via bt_le_adv_update_data().
 */
void ble_adv_update(void);

#endif /* BLE_GATT_H */
