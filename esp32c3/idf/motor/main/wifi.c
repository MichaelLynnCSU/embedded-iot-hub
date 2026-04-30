/******************************************************************************
 * \file    wifi.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   WiFi initialisation and event handling for ESP32-C3 motor node.
 ******************************************************************************/

#include "wifi.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/inet.h"
#include "wifi_secrets.h"
#include "network_config.h"
#include "wifi_tx_sweep.h"
#include "trinity_log.h"

static const char *TAG = "WIFI";

EventGroupHandle_t g_wifi_eg;

static WIFI_TX_SWEEP_T g_tx_sweep;

#define TX_SWEEP_MIN   34
#define TX_SWEEP_STEP   4
#define TX_SWEEP_MAX   84

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
        wifi_tx_sweep_on_disconnect(&g_tx_sweep);
        (void)esp_wifi_connect();
    }
    else if ((IP_EVENT == base) && (IP_EVENT_STA_GOT_IP == id))
    {
        p_event = (ip_event_got_ip_t *)p_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&p_event->ip_info.ip));
        (void)xEventGroupSetBits(g_wifi_eg, WIFI_CONNECTED_BIT);
        trinity_log_event("EVENT: WIFI_CONNECTED\n");
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

    (void)esp_wifi_set_mode(WIFI_MODE_STA);
    (void)esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    (void)esp_wifi_start();
}
