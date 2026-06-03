/******************************************************************************
 * \file    ble_adv.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    2026-06-02
 *
 * \brief   BLE advertising API for nRF52840 temp sensor node.
 *
 * \details MFG data layout (4 bytes):
 *          [0] = MFG_COMPANY_ID  (0xAE)
 *          [1] = temp_decidegc low byte   (int16 little-endian)
 *          [2] = temp_decidegc high byte
 *          [3] = batt_soc                 (0-100%)
 ******************************************************************************/

#ifndef BLE_ADV_H
#define BLE_ADV_H

#include <stdint.h>
#include <zephyr/kernel.h>

/** Message queue: temp monitor puts int16_t temp readings, BLE thread
 *  consumes and broadcasts. Two bytes per message, aligned to 2. */
extern struct k_msgq g_ble_msgq;

/**
 * \brief  BLE thread entry point. Enables BT stack, starts slow-interval
 *         broadcaster advertisement, consumes g_ble_msgq and calls
 *         ble_broadcast() on each reading or heartbeat.
 *         Never returns — run via k_thread_create().
 */
void ble_thread(void *p_unused1, void *p_unused2, void *p_unused3);

/**
 * \brief  Write temperature and battery SOC into manufacturer data and
 *         refresh advertisement atomically via bt_le_adv_update_data().
 *
 * \param  temp_decidegc  Temperature in tenths of °C (int16, little-endian).
 * \param  batt_soc       Battery SOC percent (0-100).
 * \return 0 on success, negative errno on failure.
 */
int ble_broadcast(int16_t temp_decidegc, uint8_t batt_soc);

/**
 * \brief  Write battery SOC directly into manufacturer data payload.
 *         Called from main() before BLE thread starts so the initial
 *         advertisement carries a valid battery reading.
 *
 * \param  batt_soc  Battery SOC percent (0-100).
 */
void ble_adv_set_batt(uint8_t batt_soc);

#endif /* BLE_ADV_H */
