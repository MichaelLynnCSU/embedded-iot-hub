/******************************************************************************
 * \file    ble_adv.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   BLE advertising API for ESP32-C3 PIR motion sensor node.
 ******************************************************************************/

#ifndef BLE_ADV_H
#define BLE_ADV_H

#include <stdint.h>

/**
 * \brief  Initialise and start BLE advertising.
 *         Enables BT stack, starts slow-interval broadcaster advertisement.
 *         Logs MAC address on success.
 *
 * \return 0 on success, negative errno on failure.
 */
int ble_adv_init(uint32_t motion_count, uint8_t batt_soc, uint8_t occupied);

#endif /* BLE_ADV_H */
