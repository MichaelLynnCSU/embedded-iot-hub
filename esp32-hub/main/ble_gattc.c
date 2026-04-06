/******************************************************************************
 * \file ble_gattc.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief BLE GATT client interface for ESP32 hub node.
 *
 * \details Provides a shared GATT client interface used by both the
 *          smart lock and smart light modules. Routes all GATT events
 *          to the correct device handler by conn_id matching.
 *
 *          Architecture:
 *          - Single GATT app registration (app_id = 0)
 *          - ble_gattc_if captured on ESP_GATTC_REG_EVT
 *          - All events dispatched to lock and light handlers
 *          - Each handler guards internally with its own conn_id
 *
 *          Call order:
 *          1. ble_gattc_init()  — before ble_scan_start()
 *          2. Events routed automatically via gattc_event_handler()
 ******************************************************************************/

#include "config.h"
#include "esp_log.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "ble_internal.h"
#include "trinity_log.h"

static const char *TAG = "BLE_GATTC"; /**< ESP log tag */

esp_gatt_if_t g_ble_gattc_if   = ESP_GATT_IF_NONE; /**< shared GATT interface */
bool          g_gatt_registered = false;             /**< registration flag */

/******************************************************************************
 * \brief GATT client event router.
 *
 * \param event    - GATT client event type.
 * \param gattc_if - GATT client interface handle.
 * \param p_param  - Pointer to event parameter union.
 *
 * \return void
 *
 * \details Captures the interface handle on REG_EVT. For all other
 *          events, dispatches to both lock and light handlers. Each
 *          handler guards internally with its own conn_id so sending
 *          to both is safe — only the matching one will act.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void gattc_event_handler(esp_gattc_cb_event_t event,
                                 esp_gatt_if_t gattc_if,
                                 esp_ble_gattc_cb_param_t *p_param)
{
   if (ESP_GATTC_REG_EVT == event)
   {
      if (ESP_GATT_OK == p_param->reg.status)
      {
         g_ble_gattc_if = gattc_if;
         ESP_LOGI(TAG, "GATT registered, if=%d", gattc_if);
      }
      else
      {
         ESP_LOGE(TAG, "GATT register failed, status=%d",
                  p_param->reg.status);
         trinity_log_event("EVENT: BLE_GATTC_REG_FAIL\n");
      }

      return;
   }

   ble_lock_handle_event(event, gattc_if, p_param);
   ble_light_handle_event(event, gattc_if, p_param);
}

/******************************************************************************
 * \brief Initialize the shared BLE GATT client.
 *
 * \return void
 *
 * \details Registers the GATT callback and application. Safe to call
 *          multiple times — no-op if already registered. Must be called
 *          before ble_scan_start().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_gattc_init(void)
{
   esp_err_t ret = ESP_OK; /**< esp return value */

   if (g_gatt_registered)
   {
      return;
   }

   ret = esp_ble_gattc_register_callback(gattc_event_handler);
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "GATTC callback register failed: %s",
               esp_err_to_name(ret));
      trinity_log_event("EVENT: BLE_GATTC_CB_REG_FAIL\n");
      return;
   }

   ret = esp_ble_gattc_app_register(0);
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "GATTC app register failed: %s",
               esp_err_to_name(ret));
      trinity_log_event("EVENT: BLE_GATTC_APP_REG_FAIL\n");
      return;
   }

   g_gatt_registered = true;
   ESP_LOGI(TAG, "GATTC init complete");
}
