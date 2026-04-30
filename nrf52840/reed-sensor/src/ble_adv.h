/******************************************************************************
 * \file    ble_adv.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   BLE advertising API for nRF52840 reed sensor node.
 ******************************************************************************/

#ifndef BLE_ADV_H
#define BLE_ADV_H

#include <stdint.h>
#include <zephyr/kernel.h>

/**< Message queue shared between reed monitor and BLE thread.
 *   Reed monitor puts state bytes, BLE thread consumes and broadcasts. */
extern struct k_msgq g_ble_msgq;

/**
 * \brief  BLE thread entry point. Enables BT stack, starts slow-interval
 *         broadcaster advertisement, consumes g_ble_msgq and calls
 *         ble_broadcast() on each state change or heartbeat.
 *         Never returns -- run via k_thread_create().
 */
void ble_thread(void *p_unused1, void *p_unused2, void *p_unused3);

/**
 * \brief  Write state and battery SOC into manufacturer data and refresh
 *         advertisement atomically via bt_le_adv_update_data().
 *         Does NOT stop/restart advertiser -- avoids prepare_cb race.
 *
 * \param  state     Door state (0=closed, 1=open).
 * \param  batt_soc  Battery SOC percent (0-100).
 * \return 0 on success, negative errno on failure.
 */
int ble_broadcast(uint8_t state, uint8_t batt_soc);

/**
 * \brief  Write battery SOC directly into manufacturer data payload.
 *         Called from main() before BLE thread starts so the initial
 *         advertisement carries a valid battery reading.
 *
 * \param  batt_soc  Battery SOC percent (0-100).
 */
void ble_adv_set_batt(uint8_t batt_soc);

#endif /* BLE_ADV_H */
