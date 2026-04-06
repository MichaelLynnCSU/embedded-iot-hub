/******************************************************************************
 * \file wifi_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief WiFi manager for ESP32 hub node.
 *
 * \details Manages WiFi STA connection with exponential backoff retry.
 *          Sets WIFI_CONNECTED_BIT in wifi_event_group on successful
 *          IP acquisition. All reconnection logic runs in the event
 *          handler — the task idles after initialization.
 *
 *          Backoff table: 2, 5, 10, 30, 60 seconds (defined in config.h)
 *          Credentials:   WIFI_SSID, WIFI_PASS (defined in wifi_secrets.h)
 *
 * \note    WDT fix (2026-03-21):
 *          The init sequence contains ~2 s of vTaskDelay calls, followed
 *          by ~3-5 s of WiFi association — both within a single WDT window.
 *          trinity_wdt_kick() is now called before every delay in the init
 *          path and every iteration of the idle loop (every 2 s, well within
 *          the 5 s WDT timeout).
 ******************************************************************************/

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "config.h"
#include "wifi_secrets.h"
#include "wifi_manager.h"
#include "trinity_log.h"

#define WIFI_STABILIZE_TICKS   10    /**< stabilization delay iterations         */
#define WIFI_STABILIZE_MS      100   /**< stabilization delay per iteration ms    */
#define WIFI_STACK_DELAY_MS    200   /**< delay between init steps ms             */
#define WIFI_DRIVER_DELAY_MS   300   /**< extra delay after WiFi driver init ms   */
#define WIFI_HANDLER_DELAY_MS  100   /**< delay after event handler registration ms */
#define WIFI_IDLE_DELAY_MS     2000  /**< task idle loop delay ms (must be < WDT timeout) */
#define MS_PER_SEC             1000  /**< milliseconds per second                 */

static const char *TAG = "WIFI_MGR"; /**< ESP log tag */

static EventGroupHandle_t g_wifi_eg     = NULL; /**< wifi event group handle       */
static int                g_retry_count = 0;    /**< current retry attempt count   */

/******************************************************************************
 * \brief WiFi and IP event handler.
 *
 * \param p_arg      - Unused event handler argument.
 * \param event_base - Event base (WIFI_EVENT or IP_EVENT).
 * \param event_id   - Specific event ID.
 * \param p_data     - Event data pointer (unused).
 *
 * \return void
 *
 * \details On STA_START: resets retry count and initiates connection.
 *          On DISCONNECTED: applies backoff delay and reconnects.
 *          On GOT_IP: sets WIFI_CONNECTED_BIT and resets retry count.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void wifi_event_handler(void *p_arg,
                                esp_event_base_t event_base,
                                int32_t event_id,
                                void *p_data)
{
   int idx       = 0; /**< backoff table index    */
   int delay_sec = 0; /**< backoff delay seconds  */

   (void)p_arg;
   (void)p_data;

   if ((WIFI_EVENT == event_base) &&
       (WIFI_EVENT_STA_START == event_id))
   {
      g_retry_count = 0;
      (void)esp_wifi_connect();
   }
   else if ((WIFI_EVENT == event_base) &&
            (WIFI_EVENT_STA_DISCONNECTED == event_id))
   {
      idx = (g_retry_count < WIFI_BACKOFF_TABLE_SIZE) ?
            g_retry_count : (WIFI_BACKOFF_TABLE_SIZE - 1);

      delay_sec = wifi_backoff_sec[idx];
      g_retry_count++;

      ESP_LOGW(TAG, "WiFi disconnected, retry %d in %d seconds",
               g_retry_count, delay_sec);
      trinity_log_event("EVENT: WIFI_DISCONNECTED\n");

      vTaskDelay(pdMS_TO_TICKS(delay_sec * MS_PER_SEC));
      (void)esp_wifi_connect();
   }
   else if ((IP_EVENT == event_base) &&
            (IP_EVENT_STA_GOT_IP == event_id))
   {
      ESP_LOGI(TAG, "WiFi connected");
      trinity_log_event("EVENT: WIFI_CONNECTED\n");
      g_retry_count = 0;
      (void)xEventGroupSetBits(g_wifi_eg, WIFI_CONNECTED_BIT);
   }
   else
   {
      /* unhandled event — ignore */
   }
}

/******************************************************************************
 * \brief Initialize the WiFi manager.
 *
 * \return void
 *
 * \details Currently a no-op placeholder. Called from app_main before
 *          wifi_manager_task is spawned.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void wifi_manager_init(void)
{
   ESP_LOGI(TAG, "WiFi manager initialized");
}

/******************************************************************************
 * \brief WiFi manager FreeRTOS task.
 *
 * \param p_system_eg - System event group handle (unused).
 * \param p_wifi_eg   - WiFi event group, sets WIFI_CONNECTED_BIT on connect.
 *
 * \return void
 *
 * \details Initializes netif, event loop, WiFi driver, and registers
 *          event handlers. Starts WiFi STA and enters idle loop.
 *          All reconnection logic runs in wifi_event_handler().
 *
 *          trinity_wdt_kick() is called before every blocking delay in the
 *          init path, and on every iteration of the idle loop. The idle
 *          loop delay (WIFI_IDLE_DELAY_MS = 2000 ms) is kept well below
 *          the 5 s WDT timeout.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void wifi_manager_task(EventGroupHandle_t p_system_eg,
                       EventGroupHandle_t p_wifi_eg)
{
   esp_err_t          ret      = ESP_OK;
   wifi_init_config_t cfg      = WIFI_INIT_CONFIG_DEFAULT();
   wifi_config_t      wifi_cfg = {0};
   int                i        = 0;

   (void)p_system_eg;

   g_wifi_eg = p_wifi_eg;

   ESP_LOGI(TAG, "WiFi task running on core %d", xPortGetCoreID());

   /* ---- Trinity: kick before each blocking delay in the init path ---- */
   for (i = 0; i < WIFI_STABILIZE_TICKS; i++)
   {
      trinity_wdt_kick();
      vTaskDelay(pdMS_TO_TICKS(WIFI_STABILIZE_MS));
   }

   ESP_LOGI(TAG, "Initializing netif...");
   ret = esp_netif_init();
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "netif init failed");
      return;
   }

   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(WIFI_STACK_DELAY_MS));

   ESP_LOGI(TAG, "Creating event loop...");
   ret = esp_event_loop_create_default();
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "event loop create failed");
      return;
   }

   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(WIFI_STACK_DELAY_MS));

   ESP_LOGI(TAG, "Creating default WiFi STA...");
   (void)esp_netif_create_default_wifi_sta();

   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(WIFI_STACK_DELAY_MS));

   ESP_LOGI(TAG, "Initializing WiFi driver...");
   ret = esp_wifi_init(&cfg);
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "WiFi init failed");
      return;
   }

   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(WIFI_DRIVER_DELAY_MS));

   ESP_LOGI(TAG, "Registering event handlers...");
   (void)esp_event_handler_register(WIFI_EVENT,
                                     ESP_EVENT_ANY_ID,
                                     &wifi_event_handler,
                                     NULL);
   (void)esp_event_handler_register(IP_EVENT,
                                     IP_EVENT_STA_GOT_IP,
                                     &wifi_event_handler,
                                     NULL);

   trinity_wdt_kick();
   vTaskDelay(pdMS_TO_TICKS(WIFI_HANDLER_DELAY_MS));

   ESP_LOGI(TAG, "Configuring WiFi...");

   wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
   (void)memcpy(wifi_cfg.sta.ssid,     WIFI_SSID, sizeof(wifi_cfg.sta.ssid));
   (void)memcpy(wifi_cfg.sta.password, WIFI_PASS, sizeof(wifi_cfg.sta.password));

   (void)esp_wifi_set_mode(WIFI_MODE_STA);
   (void)esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);

   ESP_LOGI(TAG, "Starting WiFi...");
   ret = esp_wifi_start();
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "WiFi start failed");
   }

   /* ---- Trinity: kick immediately after start — association begins here.
    *      The idle loop below sustains kicks during the ~3-5 s association
    *      window and indefinitely thereafter.                          ---- */
   ESP_LOGI(TAG, "WiFi init complete");

   while (1)
   {
      trinity_wdt_kick();
      vTaskDelay(pdMS_TO_TICKS(WIFI_IDLE_DELAY_MS));
   }
}
