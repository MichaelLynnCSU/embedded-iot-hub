#ifndef BLE_LIGHT_H
#define BLE_LIGHT_H

#include <stdint.h>
#include "esp_ble/esp_gattc_api.h"

void    ble_light_update_adv(uint8_t state);
uint8_t ble_light_get_state(void);
void    ble_light_try_connect(void);
void    ble_light_handle_event(esp_gattc_cb_event_t event,
                               esp_gatt_if_t gattc_if,
                               esp_ble_gattc_cb_param_t *p_param);
void    ble_send_light_command(uint8_t state);

#endif /* BLE_LIGHT_H */
