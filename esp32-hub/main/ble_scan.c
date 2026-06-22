/******************************************************************************
 * \file ble_scan.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief BLE advertisement scanner for ESP32 hub node.
 *
 * \details Parses BLE advertisements and dispatches to per-device handlers.
 *          Owns lock and light MAC discovery state.
 *
 *          Fixed devices (table-driven):
 *          - LightNF     — relay state, triggers GATT connect if pending
 *          - SmartLock   — lock state and battery, triggers GATT connect
 *
 *          Dynamic devices (prefix match, handled by sub-modules):
 *          - PIR_*       — delegated to ble_pir.c
 *          - ReedSensor* — delegated to ble_reed.c
 *          - TempSensor* — delegated to ble_temp.c
 *
 * \note    Slot table ownership (2026-06-02):
 *          PIR and reed slot tables extracted to ble_pir.c and
 *          ble_reed.c respectively, mirroring ble_temp.c pattern.
 *          ble_scan.c now owns only advertisement parsing and dispatch.
 ******************************************************************************/

#include "ble_scan.h"
#include "ble_pir.h"
#include "ble_reed.h"
#include "ble_temp.h"
#include "ble_proto.h"
#include "config.h"
#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_manager.h"
#include "ble_internal.h"
#include "vroom_bus.h"
#include <string.h>

#define ADV_TYPE_SHORT_NAME  0x08 /**< AD type: shortened local name  */
#define ADV_TYPE_FULL_NAME   0x09 /**< AD type: complete local name   */

#define MFG_LIGHT_STATE_IDX  2   /**< light state byte index         */
#define MFG_LIGHT_MIN_LEN    2   /**< min light mfg data length      */

#define MFG_LOCK_STATE_IDX   1   /**< lock state byte index          */
#define MFG_LOCK_BATT_IDX    2   /**< lock battery byte index        */
#define MFG_LOCK_MIN_LEN     3   /**< min lock mfg data length       */

static const char *TAG = "BLE_SCAN"; /**< ESP log tag */

uint8_t             lock_mac[6]     = {0};                  /**< lock device MAC   */
bool                lock_found      = false;                 /**< lock MAC found    */
esp_ble_addr_type_t lock_addr_type  = BLE_ADDR_TYPE_PUBLIC; /**< lock addr type    */

uint8_t             light_mac[6]    = {0};                  /**< light device MAC  */
bool                light_found     = false;                 /**< light MAC found   */
esp_ble_addr_type_t light_addr_type = BLE_ADDR_TYPE_PUBLIC; /**< light addr type   */

static bool g_lock_seen  = false; /**< lock first-seen flag  */
static bool g_light_seen = false; /**< light first-seen flag */

/** \brief Advertisement handler function pointer type. */
typedef void (*adv_handler_t)(const uint8_t *p_adv,
                               int len,
                               const uint8_t *p_mac,
                               uint8_t addr_type);

/** \brief Fixed device dispatch table entry. */
typedef struct
{
   const char    *p_name;  /*!< exact BLE device name to match */
   adv_handler_t  handler; /*!< handler function for this device */
} DEVICE_ENTRY_T;

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Extract BLE device name from advertisement payload.
 *
 * \param p_adv  - Pointer to raw advertisement data.
 * \param len    - Length of advertisement data in bytes.
 * \param p_out  - Output buffer for null-terminated name.
 * \param out_sz - Size of output buffer.
 *
 * \return bool - true if name was found and extracted, false otherwise.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static bool extract_name(const uint8_t *p_adv,
                          int len,
                          char *p_out,
                          int out_sz)
{
   int     i        = 0; /**< advertisement parse index */
   uint8_t flen     = 0; /**< AD structure length       */
   uint8_t ftype    = 0; /**< AD structure type         */
   int     name_len = 0; /**< extracted name length     */

   while (i < len)
   {
      flen = p_adv[i];
      i++;

      if ((0 == flen) || ((i + flen) > len))
      {
         break;
      }

      ftype = p_adv[i];

      if ((ADV_TYPE_SHORT_NAME == ftype) || (ADV_TYPE_FULL_NAME == ftype))
      {
         name_len = flen - 1;

         if (name_len >= out_sz)
         {
            name_len = out_sz - 1;
         }

         (void)memcpy(p_out, &p_adv[i + 1], name_len);
         p_out[name_len] = '\0';
         return true;
      }

      i += flen;
   }

   return false;
}

/******************************************************************************
 * \brief Find manufacturer specific data in advertisement payload.
 *
 * \param p_adv     - Pointer to raw advertisement data.
 * \param len       - Length of advertisement data in bytes.
 * \param p_out_len - Output for manufacturer data length in bytes.
 *
 * \return const uint8_t* - Pointer to manufacturer data, or NULL if not found.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static const uint8_t *find_mfg_data(const uint8_t *p_adv,
                                     int len,
                                     int *p_out_len)
{
   int     i     = 0; /**< advertisement parse index */
   uint8_t flen  = 0; /**< AD structure length       */
   uint8_t ftype = 0; /**< AD structure type         */

   while (i < len)
   {
      flen = p_adv[i];
      i++;

      if ((0 == flen) || ((i + flen) > len))
      {
         break;
      }

      ftype = p_adv[i];

      if (ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE == ftype)
      {
         *p_out_len = flen - 1;
         return &p_adv[i + 1];
      }

      i += flen;
   }

   return NULL;
}

/******************************************************************************
 * \brief Handle LightNF smart light advertisement.
 *
 * \param p_adv     - Pointer to raw advertisement data.
 * \param len       - Length of advertisement data.
 * \param p_mac     - Pointer to device MAC address.
 * \param addr_type - BLE address type.
 *
 * \return void
 *
 * \details Extracts relay state from manufacturer data. Records MAC on
 *          first seen. Triggers GATT connect attempt if command pending.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void handle_light(const uint8_t *p_adv,
                          int len,
                          const uint8_t *p_mac,
                          uint8_t addr_type)
{
   int            mfg_len    = 0;    /**< manufacturer data length    */
   const uint8_t *p_mfg     = NULL; /**< manufacturer data pointer   */
   uint8_t        new_state  = 0;   /**< extracted relay state       */
   bool           first_seen = false; /**< first advertisement flag  */
   bool           changed    = false; /**< state changed flag        */
   bool           log_due    = false; /**< periodic log due flag     */

   p_mfg = find_mfg_data(p_adv, len, &mfg_len);
   if ((NULL == p_mfg) || (mfg_len < MFG_LIGHT_MIN_LEN))
   {
      return;
   }

   new_state  = p_mfg[MFG_LIGHT_STATE_IDX];
   first_seen = !g_light_seen;
   changed    = !first_seen &&
                (new_state != (uint8_t)ble_get_light_state());
   log_due    = (ble_get_device_age_s(DEV_IDX_LIGHT) >=
                 REED_AGE_LOG_THRESHOLD);

   ble_light_update_adv(new_state);
   stamp_device(DEV_IDX_LIGHT);

   if (first_seen || changed || log_due)
   {
      g_light_seen = true;
      bus_publish_light(new_state);
      ESP_LOGI(TAG, ">>> LIGHT state=%d", new_state);
   }

   if (!light_found)
   {
      (void)memcpy(light_mac, p_mac, 6);
      light_found     = true;
      light_addr_type = addr_type;
   }

   ble_light_try_connect();
}

/******************************************************************************
 * \brief Handle SmartLock advertisement.
 *
 * \param p_adv     - Pointer to raw advertisement data.
 * \param len       - Length of advertisement data.
 * \param p_mac     - Pointer to device MAC address.
 * \param addr_type - BLE address type.
 *
 * \return void
 *
 * \details Extracts lock state and battery from manufacturer data.
 *          Records MAC on first seen. Triggers GATT connect if pending.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void handle_lock(const uint8_t *p_adv,
                         int len,
                         const uint8_t *p_mac,
                         uint8_t addr_type)
{
   int            mfg_len    = 0;    /**< manufacturer data length    */
   const uint8_t *p_mfg     = NULL; /**< manufacturer data pointer   */
   uint8_t        new_state  = 0;   /**< extracted lock state        */
   uint8_t        new_batt   = 0;   /**< extracted battery SOC       */
   bool           first_seen = false; /**< first advertisement flag  */
   bool           changed    = false; /**< state changed flag        */
   bool           log_due    = false; /**< periodic log due flag     */
   uint64_t       eid        = 0;

   p_mfg = find_mfg_data(p_adv, len, &mfg_len);
   if ((NULL == p_mfg) || (mfg_len < MFG_LOCK_MIN_LEN))
   {
      return;
   }

   new_state  = p_mfg[MFG_LOCK_STATE_IDX];
   new_batt   = p_mfg[MFG_LOCK_BATT_IDX];
   first_seen = !g_lock_seen;
   changed    = !first_seen &&
                ((new_state != (uint8_t)ble_get_lock_state()) ||
                 (new_batt  != (uint8_t)ble_lock_get_batt()));
   log_due    = (ble_get_device_age_s(DEV_IDX_LOCK) >=
                 LOCK_AGE_LOG_THRESHOLD);

   ble_lock_update_adv(new_state, new_batt);
   stamp_device(DEV_IDX_LOCK);

   if (first_seen || changed || log_due)
   {
      g_lock_seen = true;
      eid = bus_publish_lock(new_state, new_batt);
      ESP_LOGI(TAG, "[BLE_LOCK] adv state=%d batt=%d%% event_id=%llu",
               (int)new_state, (int)new_batt, (unsigned long long)eid);
   }

   if (!lock_found)
   {
      (void)memcpy(lock_mac, p_mac, 6);
      lock_found     = true;
      lock_addr_type = addr_type;
   }

   ble_lock_try_connect();
}

static const DEVICE_ENTRY_T g_device_table[] =
{
   { "LightNF",   handle_light },
   { "SmartLock", handle_lock  },
};

#define DEVICE_TABLE_SIZE \
   (sizeof(g_device_table) / sizeof(g_device_table[0])) /**< device table count */

/******************************************************************************
 * \brief Parse a BLE advertisement and dispatch to the appropriate handler.
 *
 * \param p_adv     - Pointer to raw advertisement data.
 * \param len       - Length of advertisement data.
 * \param p_mac     - Pointer to device MAC address.
 * \param addr_type - BLE address type.
 *
 * \return void
 *
 * \details PIR sensors matched by "PIR_" prefix, dispatched to ble_pir.c.
 *          Reed sensors matched by "ReedSensor" prefix, dispatched to
 *          ble_reed.c. Temp sensors matched by "TempSensor" prefix,
 *          dispatched to ble_temp.c. Fixed devices matched by exact name
 *          via g_device_table.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void parse_advertisement(const uint8_t *p_adv,
                                 int len,
                                 const uint8_t *p_mac,
                                 uint8_t addr_type)
{
   char           name[ADV_NAME_BUF_SIZE] = {0}; /**< extracted device name    */
   int            mfg_len                 = 0;   /**< manufacturer data length */
   const uint8_t *p_mfg                  = NULL; /**< manufacturer data ptr    */
   int            i                       = 0;   /**< loop index               */

   if (!extract_name(p_adv, len, name, sizeof(name)))
   {
      return;
   }

   if (0 == strncmp(name, PIR_NAME_PREFIX, PIR_NAME_PREFIX_LEN))
   {
      p_mfg = find_mfg_data(p_adv, len, &mfg_len);
      ESP_LOGI(TAG, "PIR adv mac=%02x:%02x:%02x:%02x:%02x:%02x name=%s",
               p_mac[0], p_mac[1], p_mac[2], p_mac[3], p_mac[4], p_mac[5],
               name);
      ble_pir_handle(p_mfg, mfg_len, p_mac, name);
      return;
   }

   if (0 == strncmp(name, REED_NAME_PREFIX, REED_NAME_PREFIX_LEN))
   {
      p_mfg = find_mfg_data(p_adv, len, &mfg_len);
      ble_reed_handle(p_mfg, mfg_len, p_mac, name);
      return;
   }

   if (0 == strncmp(name, TEMP_NAME_PREFIX, TEMP_NAME_PREFIX_LEN))
   {
      p_mfg = find_mfg_data(p_adv, len, &mfg_len);
      ble_temp_handle(p_mfg, mfg_len, p_mac, name);
      return;
   }

   for (i = 0; i < (int)DEVICE_TABLE_SIZE; i++)
   {
      if (0 == strcmp(name, g_device_table[i].p_name))
      {
         g_device_table[i].handler(p_adv, len, p_mac, addr_type);
         return;
      }
   }
}

/******************************************************************************
 * \brief GAP event handler for BLE scan results.
 *
 * \param event   - GAP event type.
 * \param p_param - Pointer to GAP event parameters.
 *
 * \return void
 *
 * \details Handles scan parameter set complete, scan start complete,
 *          and scan result events. Dispatches advertisements to
 *          parse_advertisement().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                               esp_ble_gap_cb_param_t *p_param)
{
   switch (event)
   {
      case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
      {
         ESP_LOGI(TAG, "Scan params set, starting scan");
         (void)esp_ble_gap_start_scanning(0);
         break;
      }

      case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
      {
         if (ESP_BT_STATUS_SUCCESS == p_param->scan_start_cmpl.status)
         {
            ESP_LOGI(TAG, "Scan started");
         }
         else
         {
            ESP_LOGE(TAG, "Scan start failed: %d",
                     p_param->scan_start_cmpl.status);
         }

         break;
      }

      case ESP_GAP_BLE_SCAN_RESULT_EVT:
      {
         if (ESP_GAP_SEARCH_INQ_RES_EVT != p_param->scan_rst.search_evt)
         {
            break;
         }

         parse_advertisement(p_param->scan_rst.ble_adv,
                              p_param->scan_rst.adv_data_len,
                              p_param->scan_rst.bda,
                              p_param->scan_rst.ble_addr_type);
         break;
      }

      default:
      {
         /* unhandled event — ignore */
         break;
      }
   }
}

/******************************************************************************
 * \brief Start BLE scanning — called by ble_manager.c after ble_gattc_init().
 *
 * \return void
 *
 * \details Registers GAP callback and sets scan parameters. Scanning
 *          begins asynchronously via ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_scan_start(void)
{
   esp_err_t ret = ESP_OK; /**< esp return value */

   static esp_ble_scan_params_t scan_params =
   {
      .scan_type          = BLE_SCAN_TYPE_ACTIVE,
      .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
      .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
      .scan_interval      = BLE_SCAN_INTERVAL,
      .scan_window        = BLE_SCAN_WINDOW,
      .scan_duplicate     = BLE_SCAN_DUPLICATE,
   };

   ret = esp_ble_gap_register_callback(gap_event_handler);
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "GAP callback register failed: %s",
               esp_err_to_name(ret));
      return;
   }

   ret = esp_ble_gap_set_scan_params(&scan_params);
   if (ESP_OK == ret)
   {
      ESP_LOGI(TAG, "Scan params configured");
   }
   else
   {
      ESP_LOGE(TAG, "Scan params failed: %s", esp_err_to_name(ret));
   }
}

/******************************************************************************
 * \brief Pre-initialize the BLE scan module.
 *
 * \return void
 *
 * \details Calls preinit for each sub-module (reed, PIR, temp).
 *          Must be called before ble_gattc_init() and ble_scan_start().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_scan_preinit(void)
{
   ble_reed_preinit();
   ble_pir_preinit();
   ble_temp_preinit();
}
