/******************************************************************************
 * \file ble_scan.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-02
 *
 * \brief BLE advertisement scanner public interface for ESP32 hub node.
 *
 * \details Exposes scanner lifecycle functions and device discovery state
 *          for lock and light MAC addresses. Called by ble_manager.c.
 *
 *          Module ownership:
 *          - lock_mac, light_mac, found flags, addr types : ble_scan.c
 *
 *          Startup sequence:
 *          1. ble_scan_preinit() — calls ble_reed_preinit(),
 *                                  ble_pir_preinit(), ble_temp_preinit()
 *          2. ble_scan_start()   — registers GAP callback, sets scan params
 ******************************************************************************/

#ifndef INCLUDE_BLE_SCAN_H_
#define INCLUDE_BLE_SCAN_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_gap_ble_api.h"

/************************* DEVICE DISCOVERY STATE *****************************/

/** Lock device MAC address — owned by ble_scan.c */
extern uint8_t             lock_mac[6];

/** Lock device found flag — owned by ble_scan.c */
extern bool                lock_found;

/** Lock device address type — owned by ble_scan.c */
extern esp_ble_addr_type_t lock_addr_type;

/** Light device MAC address — owned by ble_scan.c */
extern uint8_t             light_mac[6];

/** Light device found flag — owned by ble_scan.c */
extern bool                light_found;

/** Light device address type — owned by ble_scan.c */
extern esp_ble_addr_type_t light_addr_type;

/****************************** FUNCTION PROTOTYPES ***************************/

/******************************************************************************
 * \brief Pre-initialize the BLE scan module and all sub-modules.
 *
 * \return void
 *
 * \details Calls ble_reed_preinit(), ble_pir_preinit(), ble_temp_preinit().
 *          Must be called before ble_gattc_init() and ble_scan_start().
 ******************************************************************************/
void ble_scan_preinit(void);

/******************************************************************************
 * \brief Start BLE scanning.
 *
 * \return void
 *
 * \details Registers GAP callback and sets scan parameters. Scanning
 *          begins asynchronously via ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT.
 *          Must be called after ble_gattc_init().
 ******************************************************************************/
void ble_scan_start(void);

#endif /* INCLUDE_BLE_SCAN_H_ */
