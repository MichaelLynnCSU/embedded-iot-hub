#ifndef BLE_LOCK_H
#define BLE_LOCK_H

#include <stdint.h>
#include "esp_ble/esp_gattc_api.h"

void    ble_lock_update_adv(uint8_t state, uint8_t batt);
int     ble_lock_get_batt(void);
uint8_t ble_lock_get_state(void);
void    ble_lock_try_connect(void);
void    ble_lock_handle_event(esp_gattc_cb_event_t event,
                              esp_gatt_if_t gattc_if,
                              esp_ble_gattc_cb_param_t *p_param);
void    ble_send_lock_command(uint8_t state);

#endif /* BLE_LOCK_H */
