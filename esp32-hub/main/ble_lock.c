/******************************************************************************
 * \file ble_lock.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief BLE GATT client driver for SmartLock node.
 *
 * \details Manages the full GATT client lifecycle for the smart lock:
 *          connect, service discovery, characteristic read/write,
 *          notification subscription, battery read, and disconnect.
 *
 *          Connection flow:
 *          1. ble_scan.c calls ble_lock_try_connect() on each SmartLock adv
 *          2. connect_fn posted to connect_queue (scheduler serialises)
 *          3. do_connect_lock() opens GATT connection
 *          4. CONNECT_EVT -> MTU -> search -> char discovery -> read state
 *          5. ble_send_lock_command() writes pending state if queued
 *          6. WRITE_CHAR_EVT confirms write -> disconnect
 *
 *          Thread safety:
 *          - g_lock_mux (portMUX) guards g_lock_connecting/g_lock_connected
 *          - All state mutations outside ISR context use taskENTER_CRITICAL
 ******************************************************************************/

#include "config.h"
#include "esp_log.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_manager.h"
#include "ble_internal.h"
#include "trinity_log.h"
#include <string.h>

#define LOCK_CONN_ID_INVALID   0xFFFF  /**< sentinel for no active connection */
#define LOCK_SERVICE_UUID      0x1234  /**< lock GATT service UUID */
#define LOCK_STATE_CHAR_UUID   0x1235  /**< lock state characteristic UUID */
#define LOCK_NOTIFY_CHAR_UUID  0x1236  /**< lock notify characteristic UUID */
#define LOCK_BATT_CHAR_UUID    0x2A19  /**< battery level characteristic UUID */
#define LOCK_NOTIFY_ENABLE     1       /**< CCCD value to enable notifications */
#define LOCK_STATE_LEN         1       /**< expected state characteristic length */

static const char *TAG = "BLE_LOCK"; /**< ESP log tag */

static uint8_t g_current_lock_state = 0;   /**< last known lock state */
static int     g_lock_batt          = -1;  /**< last known battery SOC */
static int     g_pending_lock_state = -1;  /**< queued command, -1 = none */

static uint16_t g_lock_conn_id        = LOCK_CONN_ID_INVALID; /**< active conn id */
static uint16_t g_lock_service_handle = 0; /**< discovered service handle */
static uint16_t g_lock_char_handle    = 0; /**< state char handle */
static uint16_t g_lock_batt_handle    = 0; /**< battery char handle */
static bool     g_lock_connected      = false; /**< connection active flag */
static bool     g_lock_connecting     = false; /**< connection in progress flag */

static portMUX_TYPE g_lock_mux = portMUX_INITIALIZER_UNLOCKED; /**< state mutex */

static esp_bt_uuid_t g_lock_service_uuid =
{
   .len  = ESP_UUID_LEN_16,
   .uuid = {.uuid16 = LOCK_SERVICE_UUID},
}; /**< lock service UUID for discovery */

/******************************************************************************
 * \brief Internal connect function — posted to connect_queue as fn pointer.
 *
 * \return void
 *
 * \details Runs in scheduler task context. Opens GATT connection to the
 *          lock device. On failure, clears g_lock_connecting and notifies
 *          scheduler. On success, connection proceeds via CONNECT_EVT.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void do_connect_lock(void)
{
   esp_err_t ret = ESP_OK; /**< esp return value */

   if (!lock_found || (ESP_GATT_IF_NONE == g_ble_gattc_if))
   {
      ESP_LOGW(TAG, "[LOCK] Cannot connect: not found or GATT not ready");
      ble_scheduler_notify_done();
      return;
   }

   ESP_LOGI(TAG, "[LOCK] Opening connection to SmartLock");

   ret = esp_ble_gattc_open(g_ble_gattc_if,
                              lock_mac,
                              lock_addr_type,
                              true);
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "[LOCK] Open failed: %s", esp_err_to_name(ret));
      trinity_log_event("EVENT: BLE_LOCK_OPEN_FAIL\n");

      taskENTER_CRITICAL(&g_lock_mux);
      g_lock_connecting = false;
      taskEXIT_CRITICAL(&g_lock_mux);

      ble_scheduler_notify_done();
   }
}

/******************************************************************************
 * \brief Update lock state and battery from BLE advertisement data.
 *
 * \param state - Lock state byte from manufacturer data.
 * \param batt  - Battery SOC percent.
 *
 * \return void
 *
 * \details Called by ble_scan.c on every SmartLock advertisement.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_lock_update_adv(uint8_t state, uint8_t batt)
{
   g_current_lock_state = state;
   g_lock_batt          = (int)batt;
}

/******************************************************************************
 * \brief Get last known lock battery SOC.
 *
 * \return int - Battery SOC percent, -1 if unknown.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int ble_lock_get_batt(void)
{
   return g_lock_batt;
}

/******************************************************************************
 * \brief Get last known lock state.
 *
 * \return uint8_t - Lock state.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
uint8_t ble_lock_get_state(void)
{
   return g_current_lock_state;
}

/******************************************************************************
 * \brief Race-safe connection entry point — called by ble_scan.c.
 *
 * \return void
 *
 * \details Posts do_connect_lock to connect_queue if conditions are met.
 *          portMUX guard prevents double-connection race. No-op if already
 *          connected, connecting, device not found, or GATT not ready.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_lock_try_connect(void)
{
   bool do_connect = false;         /**< connection should proceed flag */
   BLE_CONNECT_REQUEST_T req = {0}; /**< connection queue request */

   taskENTER_CRITICAL(&g_lock_mux);
   if ((-1 != g_pending_lock_state) &&
       (!g_lock_connected)          &&
       (!g_lock_connecting)         &&
       (lock_found)                 &&
       (ESP_GATT_IF_NONE != g_ble_gattc_if))
   {
      g_lock_connecting = true;
      do_connect        = true;
   }
   taskEXIT_CRITICAL(&g_lock_mux);

   if (do_connect)
   {
      req.connect_fn = do_connect_lock;

      if (pdTRUE != xQueueSend(connect_queue, &req, 0))
      {
         ESP_LOGW(TAG, "[LOCK] Connect queue full, dropping request");

         taskENTER_CRITICAL(&g_lock_mux);
         g_lock_connecting = false;
         taskEXIT_CRITICAL(&g_lock_mux);
      }
      else
      {
         ESP_LOGI(TAG, "[LOCK] Queued connect request");
      }
   }
}

/******************************************************************************
 * \brief Handle service search complete — discover characteristics.
 *
 * \param gattc_if - GATT client interface handle.
 *
 * \return void
 *
 * \details Extracted from ble_lock_handle_event to keep function length
 *          under 100 lines. Enumerates all characteristics in the lock
 *          service and sets up state read, notify subscription, and
 *          battery read.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void handle_search_complete(esp_gatt_if_t gattc_if)
{
   uint16_t count = 0;                    /**< characteristic count */
   uint16_t i = 0;                        /**< loop index */
   uint16_t uuid = 0;                     /**< current char UUID */
   uint16_t notify_en = LOCK_NOTIFY_ENABLE; /**< CCCD notify value */
   esp_gatt_status_t st = ESP_GATT_OK;    /**< gatt status */
   esp_gattc_char_elem_t *p_elems = NULL; /**< char element array */
   esp_gattc_descr_elem_t desc;           /**< descriptor element */
   uint16_t desc_count = 1;               /**< descriptor count */
   esp_err_t rc = ESP_OK;                 /**< esp return value */

   esp_bt_uuid_t ccc_uuid =
   {
      .len  = ESP_UUID_LEN_16,
      .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
   };

   if (0 == g_lock_service_handle)
   {
      ESP_LOGW(TAG, "[LOCK] Service not found, disconnecting");
      (void)esp_ble_gattc_close(gattc_if, g_lock_conn_id);
      ble_scheduler_notify_done();
      return;
   }

   st = esp_ble_gattc_get_attr_count(gattc_if,
                                      g_lock_conn_id,
                                      ESP_GATT_DB_CHARACTERISTIC,
                                      g_lock_service_handle,
                                      0xFFFF, 0, &count);
   if ((ESP_GATT_OK != st) || (0 == count))
   {
      return;
   }

   p_elems = malloc(sizeof(esp_gattc_char_elem_t) * count);
   if (NULL == p_elems)
   {
      return;
   }

   st = esp_ble_gattc_get_all_char(gattc_if,
                                    g_lock_conn_id,
                                    g_lock_service_handle,
                                    0xFFFF, p_elems, &count, 0);
   if (ESP_GATT_OK == st)
   {
      for (i = 0; i < count; i++)
      {
         if (ESP_UUID_LEN_16 != p_elems[i].uuid.len)
         {
            continue;
         }

         uuid = p_elems[i].uuid.uuid.uuid16;

         if (LOCK_STATE_CHAR_UUID == uuid)
         {
            g_lock_char_handle = p_elems[i].char_handle;
            ESP_LOGI(TAG, "[LOCK] State char found, reading...");
            (void)esp_ble_gattc_read_char(gattc_if,
                                           g_lock_conn_id,
                                           g_lock_char_handle,
                                           ESP_GATT_AUTH_REQ_NONE);
         }
         else if (LOCK_NOTIFY_CHAR_UUID == uuid)
         {
            rc = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                          g_lock_conn_id,
                                                          p_elems[i].char_handle,
                                                          ccc_uuid,
                                                          &desc,
                                                          &desc_count);
            if ((ESP_OK == rc) && (0 < desc_count))
            {
               (void)esp_ble_gattc_write_char_descr(gattc_if,
                                                     g_lock_conn_id,
                                                     desc.handle,
                                                     sizeof(notify_en),
                                                     (uint8_t *)&notify_en,
                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                     ESP_GATT_AUTH_REQ_NONE);
            }
         }
         else if (LOCK_BATT_CHAR_UUID == uuid)
         {
            g_lock_batt_handle = p_elems[i].char_handle;
            ESP_LOGI(TAG, "[LOCK] Battery char found, reading...");
            (void)esp_ble_gattc_read_char(gattc_if,
                                           g_lock_conn_id,
                                           g_lock_batt_handle,
                                           ESP_GATT_AUTH_REQ_NONE);
         }
         else
         {
            /* unknown characteristic — skip */
         }
      }
   }

   free(p_elems);
   p_elems = NULL;
}

/******************************************************************************
 * \brief GATT event handler — called by ble_gattc.c router.
 *
 * \param event    - GATT client event type.
 * \param gattc_if - GATT client interface handle.
 * \param p_param  - Pointer to GATT event parameter union.
 *
 * \return void
 *
 * \details Guards on g_lock_conn_id — light events are silently ignored.
 *          Handles full GATT lifecycle: connect, MTU, search, read,
 *          notify, write, and disconnect.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_lock_handle_event(esp_gattc_cb_event_t event,
                            esp_gatt_if_t gattc_if,
                            esp_ble_gattc_cb_param_t *p_param)
{
   switch (event)
   {
      case ESP_GATTC_CONNECT_EVT:
      {
         if (0 != memcmp(p_param->connect.remote_bda, lock_mac, 6))
         {
            break;
         }

         ESP_LOGI(TAG, "[LOCK] Connected, conn_id=%d",
                  p_param->connect.conn_id);
         g_lock_conn_id    = p_param->connect.conn_id;
         g_lock_connected  = true;
         g_lock_connecting = false;
         (void)esp_ble_gattc_send_mtu_req(gattc_if, g_lock_conn_id);
         break;
      }

      case ESP_GATTC_OPEN_EVT:
      {
         if (LOCK_CONN_ID_INVALID == g_lock_conn_id)
         {
            break;
         }

         if (p_param->open.conn_id != g_lock_conn_id)
         {
            break;
         }

         if (ESP_GATT_OK != p_param->open.status)
         {
            ESP_LOGE(TAG, "[LOCK] Open failed, status=%d",
                     p_param->open.status);
            trinity_log_event("EVENT: BLE_LOCK_OPEN_FAIL\n");

            taskENTER_CRITICAL(&g_lock_mux);
            g_lock_connected  = false;
            g_lock_connecting = false;
            taskEXIT_CRITICAL(&g_lock_mux);

            g_lock_conn_id = LOCK_CONN_ID_INVALID;
            ble_scheduler_notify_done();
         }

         break;
      }

      case ESP_GATTC_CFG_MTU_EVT:
      {
         if ((p_param->cfg_mtu.conn_id != g_lock_conn_id) ||
             (LOCK_CONN_ID_INVALID == g_lock_conn_id))
         {
            break;
         }

         ESP_LOGI(TAG, "[LOCK] MTU=%d, searching service",
                  p_param->cfg_mtu.mtu);
         (void)esp_ble_gattc_search_service(gattc_if,
                                             g_lock_conn_id,
                                             &g_lock_service_uuid);
         break;
      }

      case ESP_GATTC_SEARCH_RES_EVT:
      {
         if ((p_param->search_res.conn_id != g_lock_conn_id) ||
             (LOCK_CONN_ID_INVALID == g_lock_conn_id))
         {
            break;
         }

         if ((ESP_UUID_LEN_16 ==
              p_param->search_res.srvc_id.uuid.len) &&
             (LOCK_SERVICE_UUID ==
              p_param->search_res.srvc_id.uuid.uuid.uuid16))
         {
            g_lock_service_handle = p_param->search_res.start_handle;
            ESP_LOGI(TAG, "[LOCK] Service found");
         }

         break;
      }

      case ESP_GATTC_SEARCH_CMPL_EVT:
      {
         if ((p_param->search_cmpl.conn_id != g_lock_conn_id) ||
             (LOCK_CONN_ID_INVALID == g_lock_conn_id))
         {
            break;
         }

         handle_search_complete(gattc_if);
         break;
      }

      case ESP_GATTC_READ_CHAR_EVT:
      {
         if ((p_param->read.conn_id != g_lock_conn_id) ||
             (LOCK_CONN_ID_INVALID == g_lock_conn_id))
         {
            break;
         }

         if ((ESP_GATT_OK == p_param->read.status) &&
             (LOCK_STATE_LEN == p_param->read.value_len))
         {
            if ((p_param->read.handle == g_lock_batt_handle) &&
                (0 != g_lock_batt_handle))
            {
               g_lock_batt = (int)p_param->read.value[0];
               ESP_LOGI(TAG, "[LOCK] Battery=%d%%", g_lock_batt);
            }
            else
            {
               g_current_lock_state = p_param->read.value[0];
               stamp_device(DEV_IDX_LOCK);
               ESP_LOGI(TAG, "[LOCK] State=%d batt=%d%%",
                        g_current_lock_state, g_lock_batt);
            }
         }

         if (-1 != g_pending_lock_state)
         {
            ble_send_lock_command((uint8_t)g_pending_lock_state);
         }

         break;
      }

      case ESP_GATTC_NOTIFY_EVT:
      {
         if ((p_param->notify.conn_id != g_lock_conn_id) ||
             (LOCK_CONN_ID_INVALID == g_lock_conn_id))
         {
            break;
         }

         stamp_device(DEV_IDX_LOCK);

         if (LOCK_STATE_LEN == p_param->notify.value_len)
         {
            g_current_lock_state = p_param->notify.value[0];
            ESP_LOGI(TAG, "[LOCK] State notified=%d",
                     g_current_lock_state);
         }

         break;
      }

      case ESP_GATTC_WRITE_CHAR_EVT:
      {
         if ((p_param->write.conn_id != g_lock_conn_id) ||
             (LOCK_CONN_ID_INVALID == g_lock_conn_id))
         {
            break;
         }

         if (ESP_GATT_OK == p_param->write.status)
         {
            ESP_LOGI(TAG, "[LOCK] Write confirmed, disconnecting");
            g_pending_lock_state = -1;
         }
         else
         {
            ESP_LOGW(TAG, "[LOCK] Write failed, status=%d",
                     p_param->write.status);
         }

         (void)esp_ble_gattc_close(gattc_if, g_lock_conn_id);
         break;
      }

      case ESP_GATTC_WRITE_DESCR_EVT:
      {
         if ((p_param->write.conn_id != g_lock_conn_id) ||
             (LOCK_CONN_ID_INVALID == g_lock_conn_id))
         {
            break;
         }

         if (ESP_GATT_OK == p_param->write.status)
         {
            ESP_LOGI(TAG, "[LOCK] Notifications subscribed");
         }
         else
         {
            ESP_LOGW(TAG, "[LOCK] Notify subscribe failed, status=%d",
                     p_param->write.status);
         }

         break;
      }

      case ESP_GATTC_DISCONNECT_EVT:
      {
         if ((p_param->disconnect.conn_id != g_lock_conn_id) ||
             (LOCK_CONN_ID_INVALID == g_lock_conn_id))
         {
            break;
         }

         ESP_LOGW(TAG, "[LOCK] Disconnected, reason=0x%x",
                  p_param->disconnect.reason);
         trinity_log_event("EVENT: BLE_LOCK_DISCONNECTED\n");

         taskENTER_CRITICAL(&g_lock_mux);
         g_lock_connected  = false;
         g_lock_connecting = false;
         taskEXIT_CRITICAL(&g_lock_mux);

         g_lock_char_handle    = 0;
         g_lock_service_handle = 0;
         g_lock_batt_handle    = 0;
         g_lock_conn_id        = LOCK_CONN_ID_INVALID;

         ble_scheduler_notify_done();
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
 * \brief Send a lock state command via GATT write.
 *
 * \param state - Lock state to write (0=unlock, 1=lock).
 *
 * \return void
 *
 * \details If not connected or char handle not yet discovered, queues
 *          the state in g_pending_lock_state for delivery after the
 *          next connection and characteristic read completes.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_send_lock_command(uint8_t state)
{
   esp_err_t ret = ESP_OK; /**< esp return value */

   if (!g_lock_connected || (0 == g_lock_char_handle))
   {
      ESP_LOGW(TAG, "[LOCK] Not connected, queuing state=%d", state);
      g_pending_lock_state = (int)state;
      return;
   }

   ret = esp_ble_gattc_write_char(g_ble_gattc_if,
                                   g_lock_conn_id,
                                   g_lock_char_handle,
                                   LOCK_STATE_LEN,
                                   &state,
                                   ESP_GATT_WRITE_TYPE_RSP,
                                   ESP_GATT_AUTH_REQ_NONE);
   if (ESP_OK == ret)
   {
      ESP_LOGI(TAG, "[LOCK] Command sent: %d", state);
      g_pending_lock_state = -1;
   }
   else
   {
      ESP_LOGE(TAG, "[LOCK] Write failed: %s", esp_err_to_name(ret));
      g_pending_lock_state = (int)state;
   }
}
