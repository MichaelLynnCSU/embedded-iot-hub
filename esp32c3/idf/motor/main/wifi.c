/******************************************************************************
 * \file    wifi.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   WiFi initialisation and event handling for ESP32-C3 motor node.
 *
 * \note    TX sweep fix (2026-05-04):
 *          wifi_tx_sweep_on_disconnect() calls esp_wifi_set_max_tx_power()
 *          which touches RF hardware and can exceed the 300ms INT WDT
 *          timeout when called from the WiFi event handler (interrupt
 *          context), causing TG1WDT_SYS_RST.
 *          Fix: event handler sets g_sweep_pending flag only. RF power
 *          change is applied by wifi_service_tx_sweep(), called from
 *          tcp_rx_task loop (task context, no INT WDT constraint).
 *
 * \note    Reconnect fix (2026-05-04):
 *          esp_wifi_connect() called from WIFI_EVENT_STA_DISCONNECTED handler
 *          triggers internal driver state-machine work that holds the CPU
 *          >300ms on rapid beacon-drop events from certain APs, causing
 *          TG0WDT_SYS_RST / TG1WDT_SYS_RST (interrupt watchdog).
 *          Fix: event handler sets g_reconnect_pending flag only.
 *          esp_wifi_connect() is called from wifi_service_reconnect(),
 *          consumed from task context alongside wifi_service_tx_sweep().
 *
 * \note    Event bit fix (2026-05-04):
 *          WIFI_CONNECTED_BIT must be cleared on WIFI_EVENT_STA_DISCONNECTED
 *          so tcp_rx_task re-enters its WiFi wait loop and stops trying to
 *          accept() on a dead interface. Previously the bit was never cleared,
 *          so tcp_rx_task spun on accept() forever after a beacon drop while
 *          the reconnect never fired.
 *
 * \note    PS timing fix (2026-05-04):
 *          esp_wifi_set_ps() moved to before esp_wifi_start() so the mode
 *          is set before the driver begins association. When called after
 *          start(), the AP has already negotiated PS during the association
 *          exchange and ignores the subsequent override.
 *
 * \note    Reconnect retry fix (2026-05-04):
 *          esp_wifi_connect() returns ESP_ERR_WIFI_CONN if called while the
 *          driver state machine is still in the disassoc transition (0xc800).
 *          The original flag-clear-then-call pattern consumed g_reconnect_pending
 *          before confirming success, so a failed connect was never retried and
 *          the device hung indefinitely. Fix: retry with 200ms backoff up to
 *          If the driver is not ready (ESP_ERR_WIFI_CONN), the flag is left
 *          set and retried on the next wait_for_wifi() tick (1s later).
 *          No busy-loop, no exhaustion -- retries until the driver accepts it.
 *
 * \note    Power / light sleep fix (2026-05-05):
 *          WIFI_PS_NONE replaced with WIFI_PS_MAX_MODEM before esp_wifi_start()
 *          so the AP negotiates modem sleep during association. Previously
 *          WIFI_PS_NONE kept the radio fully active (wifi:pm type:0) drawing
 *          ~125mA constant. esp_pm_configure() is called on GOT_IP to activate
 *          light sleep with CPU power-down, dropping idle draw from ~56mA to
 *          the target ~2-5mA. CONFIG_PM_ENABLE, CONFIG_PM_POWER_DOWN_CPU_IN_
 *          LIGHT_SLEEP, and CONFIG_FREERTOS_USE_TICKLESS_IDLE must all be set
 *          in sdkconfig (verified 2026-05-05).
 ******************************************************************************/

#include "wifi.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_pm.h"
#include "lwip/inet.h"
#include "wifi_secrets.h"
#include "network_config.h"
#include "wifi_tx_sweep.h"
#include "trinity_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "WIFI";

EventGroupHandle_t g_wifi_eg;

static WIFI_TX_SWEEP_T g_tx_sweep;

/* Set by event handler (interrupt context), cleared by wifi_service_tx_sweep()
 * (task context). Volatile -- written and read from different contexts. */
static volatile bool g_sweep_pending     = false;

/* Set by event handler (interrupt context), cleared by
 * wifi_service_reconnect() (task context) only after esp_wifi_connect()
 * succeeds. Retried with backoff if driver state machine is not ready. */
static volatile bool g_reconnect_pending = false;

#define TX_SWEEP_MIN            34
#define TX_SWEEP_STEP            4
#define TX_SWEEP_MAX            84


/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Service TX power sweep from task context.
 *
 * \return void
 *
 * \details Must be called regularly from a FreeRTOS task (not ISR/event
 *          handler). Applies the pending TX power step if the event handler
 *          set g_sweep_pending. Safe to call every loop iteration.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void wifi_service_tx_sweep(void)
{
    if (!g_sweep_pending) { return; }
    g_sweep_pending = false;
    wifi_tx_sweep_on_disconnect(&g_tx_sweep);
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Service pending WiFi reconnect from task context.
 *
 * \return void
 *
 * \details Must be called regularly from a FreeRTOS task (not ISR/event
 *          handler). When g_reconnect_pending is set, attempts
 *          esp_wifi_connect() once per call. The driver state machine may still
 *          be in the disassoc transition -- if so, the flag is left set and
 *          be in the disassoc transition (0xc800) when the disconnect event
 *          fires; esp_wifi_connect() returns ESP_ERR_WIFI_CONN in that case
 *          and must be retried rather than silently dropped.
 *
 *          The flag is cleared only on a successful (ESP_OK) return or after
 *          all attempts are exhausted. A new disconnect event will re-set it.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void wifi_service_reconnect(void)
{
    esp_err_t err;
    static int s_attempt = 0;

    if (!g_reconnect_pending)
    {
        return;
    }

    s_attempt++;
    ESP_LOGI(TAG, "[RECONNECT] attempt %d: calling esp_wifi_connect()", s_attempt);

    err = esp_wifi_connect();

    if (ESP_OK == err)
    {
        ESP_LOGI(TAG, "[RECONNECT] esp_wifi_connect() accepted (attempt %d)", s_attempt);
        s_attempt = 0;
        g_reconnect_pending = false;
        return;
    }

    if (ESP_ERR_WIFI_CONN == err)
    {
        /* 0x3002 -- driver state machine not ready (still at 0xc800).
         * Leave flag set, retry next tick. */
        ESP_LOGW(TAG, "[RECONNECT] driver not ready (0x%x), will retry (attempt %d)",
                 (unsigned)err, s_attempt);
        return;
    }

    /* Any other error -- log full details, keep retrying. */
    ESP_LOGE(TAG, "[RECONNECT] esp_wifi_connect() unexpected err=0x%x attempt=%d",
             (unsigned)err, s_attempt);
}

/*----------------------------------------------------------------------------*/

static void wifi_event_handler(void *p_arg,
                                esp_event_base_t base,
                                int32_t id,
                                void *p_data)
{
    ip_event_got_ip_t *p_event = NULL;

    (void)p_arg;

    if ((WIFI_EVENT == base) && (WIFI_EVENT_STA_START == id))
    {
        (void)esp_wifi_connect();
    }
    else if ((WIFI_EVENT == base) && (WIFI_EVENT_STA_DISCONNECTED == id))
    {
        wifi_event_sta_disconnected_t *p_disc =
            (wifi_event_sta_disconnected_t *)p_data;
        ESP_LOGW(TAG, "[EVENT] STA_DISCONNECTED reason=%d (0x%02x)",
                 p_disc ? (int)p_disc->reason : -1,
                 p_disc ? (unsigned)p_disc->reason : 0);

        /* Clear connected bit immediately so tcp_rx_task stops accepting
         * and re-enters its WiFi wait loop. */
        (void)xEventGroupClearBits(g_wifi_eg, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "[EVENT] WIFI_CONNECTED_BIT cleared");

        /* Flags only -- both RF power change and reconnect are deferred to
         * task context to avoid INT WDT violation on rapid beacon drops. */
        g_sweep_pending     = true;
        g_reconnect_pending = true;
        ESP_LOGI(TAG, "[EVENT] g_reconnect_pending=true g_sweep_pending=true");
    }
    else if ((IP_EVENT == base) && (IP_EVENT_STA_GOT_IP == id))
    {
        p_event = (ip_event_got_ip_t *)p_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&p_event->ip_info.ip));
        ESP_LOGI(TAG, "[EVENT] GOT_IP -- setting WIFI_CONNECTED_BIT");
        wifi_tx_sweep_reset(&g_tx_sweep);   /* reset TX power after successful reconnect */
        (void)xEventGroupSetBits(g_wifi_eg, WIFI_CONNECTED_BIT);
        trinity_log_event("EVENT: WIFI_CONNECTED\n");

        /* Activate light sleep now that WiFi is up. CPU powers down between
         * FreeRTOS ticks; WiFi stack wakes for beacons automatically.
         * max_freq_mhz=80 reduces active-phase draw; min_freq_mhz=10 is the
         * lowest the ESP32-C3 supports. Requires CONFIG_PM_ENABLE,
         * CONFIG_PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP, and
         * CONFIG_FREERTOS_USE_TICKLESS_IDLE in sdkconfig. */
        esp_pm_config_t pm_cfg = {
            .max_freq_mhz       = 80,
            .min_freq_mhz       = 40,
            .light_sleep_enable = true,
        };
        esp_err_t pm_err = esp_pm_configure(&pm_cfg);
        if (ESP_OK == pm_err)
        {
            ESP_LOGI(TAG, "[PM] light sleep enabled (max=80MHz min=10MHz)");
        }
        else
        {
            ESP_LOGE(TAG, "[PM] esp_pm_configure failed: 0x%x", (unsigned)pm_err);
        }
    }
}

/*----------------------------------------------------------------------------*/

void wifi_init(void)
{
    esp_netif_t        *p_netif  = NULL;
    esp_netif_ip_info_t ip_info;
    wifi_init_config_t  cfg      = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t       wifi_cfg = {0};

    g_wifi_eg = xEventGroupCreate();

    wifi_tx_sweep_init(&g_tx_sweep, TX_SWEEP_MIN, TX_SWEEP_STEP, TX_SWEEP_MAX);

    (void)esp_netif_init();
    (void)esp_event_loop_create_default();

    p_netif = esp_netif_create_default_wifi_sta();
    (void)esp_netif_dhcpc_stop(p_netif);

    (void)memset(&ip_info, 0, sizeof(ip_info));
    ip_info.ip.addr      = ipaddr_addr(STATIC_IP);
    ip_info.gw.addr      = ipaddr_addr(STATIC_GW);
    ip_info.netmask.addr = ipaddr_addr(STATIC_NETMASK);
    (void)esp_netif_set_ip_info(p_netif, &ip_info);

    (void)esp_wifi_init(&cfg);
    (void)esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     wifi_event_handler, NULL);
    (void)esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                     wifi_event_handler, NULL);

    strncpy((char *)wifi_cfg.sta.ssid, WIFI_SSID,
            sizeof(wifi_cfg.sta.ssid) - 1);
    wifi_cfg.sta.ssid[sizeof(wifi_cfg.sta.ssid) - 1] = '\0';
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASS,
            sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.password[sizeof(wifi_cfg.sta.password) - 1] = '\0';

    /* listen_interval = 1: check every beacon, not every 3.
     * This AP ignores WIFI_PS_NONE and forces li=3; setting it explicitly
     * in the STA config overrides that during association. */
    wifi_cfg.sta.listen_interval = 1;

    (void)esp_wifi_set_mode(WIFI_MODE_STA);
    (void)esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);

    /* WIFI_PS_MAX_MODEM set before esp_wifi_start() so the AP negotiates
     * modem sleep during association. Previously WIFI_PS_NONE was set here
     * which kept the radio fully active (wifi:pm type:0, ~125mA constant).
     * Must be set before start() -- the AP ignores overrides after
     * association has completed. */
    (void)esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

    (void)esp_wifi_start();
}
