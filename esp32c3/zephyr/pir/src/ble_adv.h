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
 *         Increments s_tx_id (per-wake-session counter, RAM only) and stamps
 *         it into mfg_data[8..9] little-endian before advertising starts.
 *         Logs tx_id, motion count, and battery SOC on success.
 *
 *         tx_id is NOT a global unique message ID. Correlation key is
 *         (MAC + tx_id) within a bounded time window.
 *
 * \return 0 on success, negative errno on failure.
 */
int ble_adv_init(uint32_t motion_count, uint8_t batt_soc, uint8_t occupied);

#endif /* BLE_ADV_H */
