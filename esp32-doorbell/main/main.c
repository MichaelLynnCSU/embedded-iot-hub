/******************************************************************************
 * \file main.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief ESP32 doorbell CAM — button-triggered JPEG capture and push to BBB.
 *
 * \details Board: AI-Thinker ESP32-CAM (OV2640).
 *          Pinout from AI-Thinker schematic.
 *
 *          Flow:
 *          boot -> wifi up -> wait for GPIO13 button press
 *          button press -> generate event_id
 *                       -> send UDP event to hub (Lane A, authoritative)
 *                       -> capture JPEG
 *                       -> connect to BBB:9091 -> send header+JPEG -> close
 *
 *          TCP connection is opened per-trigger and closed after send.
 *          Button is debounced in software (DEBOUNCE_MS).
 *          GPIO13 pullup disabled — external LED+resistor acts as pullup.
 *
 * \note    Separate from S3 PIR cam which streams MJPEG to BBB:9090.
 *          Doorbell sends single JPEG to BBB:9091 on each button press.
 *          Up to MAX_DOORBELL_CAMS (4) devices share port 9091.
 *          Device ID set at build time: idf.py -DDOORBELL_ID=N build
 *
 * \note    Non-blocking connect (2026-06-07):
 *          tcp_connect_once() retries 3x then drops frame and returns to
 *          waiting. Always captures fresh image on next button press.
 *
 * \note    Internal pullup disabled (2026-06-07):
 *          GPIO_PULLUP_DISABLE on GPIO13. Internal ~45kΩ pullup caused LED
 *          to glow dimly when button not pressed. External LED+resistor
 *          circuit provides the pullup instead.
 *
 * \note    Dual-lane event architecture (2026-06-07):
 *          Lane A: UDP event  → hub   (intent, authoritative)
 *          Lane B: TCP JPEG   → BBB   (payload, best-effort)
 *          Neither lane depends on the other for correctness.
 *          event_id ties both lanes together as a join key at the BBB.
 ******************************************************************************/

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_camera.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "wifi_secrets.h"
#include "cam_logic.h"

static const char *TAG = "DOORBELL";

/*---------------------------------------------------------------------------*/
/* Globals                                                                     */
/*---------------------------------------------------------------------------*/

static SemaphoreHandle_t g_trigger_sem  = NULL;
static int               g_tcp_sock     = -1;
static bool              g_wifi_up      = false;

/* event_id generation state — set once after WiFi up */
static uint32_t          g_mac_tail     = 0;
static uint32_t          g_boot_ms      = 0;
static uint16_t          g_event_seq    = 0;

/*---------------------------------------------------------------------------*/
/* Camera init                                                                 */
/*---------------------------------------------------------------------------*/

static esp_err_t camera_init(void)
{
    camera_config_t config = {
        .pin_pwdn       = CAM_PIN_PWDN,
        .pin_reset      = CAM_PIN_RESET,
        .pin_xclk       = CAM_PIN_XCLK,
        .pin_sccb_sda   = CAM_PIN_SIOD,
        .pin_sccb_scl   = CAM_PIN_SIOC,
        .pin_d7         = CAM_PIN_D7,
        .pin_d6         = CAM_PIN_D6,
        .pin_d5         = CAM_PIN_D5,
        .pin_d4         = CAM_PIN_D4,
        .pin_d3         = CAM_PIN_D3,
        .pin_d2         = CAM_PIN_D2,
        .pin_d1         = CAM_PIN_D1,
        .pin_d0         = CAM_PIN_D0,
        .pin_vsync      = CAM_PIN_VSYNC,
        .pin_href       = CAM_PIN_HREF,
        .pin_pclk       = CAM_PIN_PCLK,
        .xclk_freq_hz   = 20000000,
        .ledc_timer     = LEDC_TIMER_0,
        .ledc_channel   = LEDC_CHANNEL_0,
        .pixel_format   = PIXFORMAT_JPEG,
        .frame_size     = FRAMESIZE_QQVGA,
        .jpeg_quality   = 12,
        .fb_count       = 1,
        .fb_location    = CAMERA_FB_IN_DRAM,
        .grab_mode      = CAMERA_GRAB_WHEN_EMPTY,
    };

    esp_err_t err = esp_camera_init(&config);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    return ESP_OK;
}

/*---------------------------------------------------------------------------*/
/* WiFi                                                                        */
/*---------------------------------------------------------------------------*/

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (WIFI_EVENT == base && WIFI_EVENT_STA_START == id)
    {
        esp_wifi_connect();
    }
    else if (WIFI_EVENT == base && WIFI_EVENT_STA_DISCONNECTED == id)
    {
        g_wifi_up = false;
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
        esp_wifi_connect();
    }
    else if (IP_EVENT == base && IP_EVENT_STA_GOT_IP == id)
    {
        ip_event_got_ip_t *p_event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&p_event->ip_info.ip));

        /* Seed event_id state once — MAC stable after WiFi association */
        uint8_t mac[6] = {0};
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        g_mac_tail = ((uint32_t)mac[3] << 16) |
                     ((uint32_t)mac[4] <<  8) |
                     ((uint32_t)mac[5]);
        g_boot_ms  = (uint32_t)(esp_timer_get_time() / 1000ULL);

        g_wifi_up = true;
    }
}

static void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                               wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                               wifi_event_handler, NULL);

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid,     WIFI_SSID, 32);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, 64);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}


/*---------------------------------------------------------------------------*/
/* Lane A — UDP event to hub (authoritative, fires before image capture)      */
/*---------------------------------------------------------------------------*/

static void send_udp_event(uint64_t event_id)
{
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(HUB_UDP_PORT);
    inet_pton(AF_INET, HUB_HOST, &addr.sin_addr);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        ESP_LOGW(TAG, "UDP event: socket() failed");
        return;
    }

    char    buf[160];
    int     len;
    uint64_t ts_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);

    len = snprintf(buf, sizeof(buf),
        "{\"device_id\":%d,\"device_type\":\"doorbell\","
        "\"event_type\":\"press\","
        "\"event_id\":\"%08lx%08lx\","
        "\"timestamp_ms\":%llu}",
        DOORBELL_ID,
        (unsigned long)(event_id >> 32),
        (unsigned long)(event_id & 0xFFFFFFFFUL),
        (unsigned long long)ts_ms);

    sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
    close(sock);

    ESP_LOGI(TAG, "UDP event sent event_id=%08lx%08lx",
             (unsigned long)(event_id >> 32),
             (unsigned long)(event_id & 0xFFFFFFFFUL));
}

/*---------------------------------------------------------------------------*/
/* Lane B — TCP JPEG to BBB (payload, best-effort)                            */
/*---------------------------------------------------------------------------*/

static bool tcp_connect_once(void)
{
    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(BBB_PORT);
    inet_pton(AF_INET, BBB_HOST, &addr.sin_addr);

    if (g_tcp_sock >= 0) { close(g_tcp_sock); g_tcp_sock = -1; }

    g_tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (0 > g_tcp_sock) { return false; }

    if (0 == connect(g_tcp_sock, (struct sockaddr *)&addr, sizeof(addr)))
    {
        ESP_LOGI(TAG, "Connected to BBB %s:%d (device_id=%d)",
                 BBB_HOST, BBB_PORT, DOORBELL_ID);
        return true;
    }

    close(g_tcp_sock);
    g_tcp_sock = -1;
    return false;
}

static bool send_jpeg_to_bbb(camera_fb_t *p_fb, uint64_t event_id)
{
    uint8_t hdr_buf[CAM_HEADER_SIZE];
    cam_pack_header(hdr_buf, (uint8_t)DOORBELL_ID, event_id,
                    (uint32_t)p_fb->len);

    if (CAM_HEADER_SIZE != send(g_tcp_sock, hdr_buf, CAM_HEADER_SIZE, 0))
    {
        return false;
    }

    size_t sent = 0;
    while (sent < p_fb->len)
    {
        int n = send(g_tcp_sock, p_fb->buf + sent, p_fb->len - sent, 0);
        if (0 >= n) { return false; }
        sent += (size_t)n;
    }

    ESP_LOGI(TAG, "Sent JPEG %zu bytes event_id=%08lx%08lx",
             p_fb->len,
             (unsigned long)(event_id >> 32),
             (unsigned long)(event_id & 0xFFFFFFFFUL));
    return true;
}

/*---------------------------------------------------------------------------*/
/* Button ISR                                                                  */
/*---------------------------------------------------------------------------*/

static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(g_trigger_sem, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
}

static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,   /* external LED+resistor pullup */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
}

/*---------------------------------------------------------------------------*/
/* Capture task                                                                */
/*---------------------------------------------------------------------------*/

static void capture_task(void *arg)
{
    while (!g_wifi_up) { vTaskDelay(pdMS_TO_TICKS(500)); }

    ESP_LOGI(TAG, "Ready — waiting for doorbell button press (device_id=%d)",
             DOORBELL_ID);

    while (1)
    {
        xSemaphoreTake(g_trigger_sem, portMAX_DELAY);

        /* Debounce */
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));

        /* Drain bounce */
        while (xSemaphoreTake(g_trigger_sem, 0) == pdTRUE) {}

        ESP_LOGI(TAG, "Doorbell pressed — generating event");

        /* Generate event_id — join key for both lanes */
        uint64_t event_id = cam_make_event_id(g_mac_tail, g_boot_ms,
                                               g_event_seq++);

        /* Lane A — fire UDP event to hub immediately, before image capture.
         * Hub correctness never depends on image delivery. */
        send_udp_event(event_id);

        /* Lane B — capture JPEG and push to BBB with event_id in header */
        ESP_LOGI(TAG, "Capturing JPEG");

        camera_fb_t *p_fb = NULL;
        for (int i = 0; i < 5; i++)
        {
            p_fb = esp_camera_fb_get();
            if (NULL != p_fb) { break; }
            ESP_LOGW(TAG, "Capture failed, retry %d", i + 1);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (NULL == p_fb)
        {
            ESP_LOGE(TAG, "Camera capture failed after retries");
            continue;
        }

        ESP_LOGI(TAG, "Captured JPEG %zu bytes", p_fb->len);

        bool connected = false;
        for (int i = 0; i < 3; i++)
        {
            if (tcp_connect_once()) { connected = true; break; }
            ESP_LOGW(TAG, "Connect attempt %d failed", i + 1);
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
        }

        if (!connected)
        {
            ESP_LOGE(TAG, "BBB unreachable, dropping frame");
            esp_camera_fb_return(p_fb);
            continue;
        }

        if (!send_jpeg_to_bbb(p_fb, event_id))
        {
            ESP_LOGW(TAG, "JPEG send failed");
        }

        esp_camera_fb_return(p_fb);
        close(g_tcp_sock);
        g_tcp_sock = -1;
    }
}

/*---------------------------------------------------------------------------*/
/* Heartbeat                                                                   */
/*---------------------------------------------------------------------------*/

#define HEARTBEAT_INTERVAL_MS  30000   /* 30s — tune to taste */

static void send_heartbeat(void)
{
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(HUB_UDP_PORT);
    inet_pton(AF_INET, HUB_HOST, &addr.sin_addr);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { return; }

    char    buf[128];
    uint64_t ts_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);

    int len = snprintf(buf, sizeof(buf),
        "{\"device_id\":%d,\"device_type\":\"doorbell\","
        "\"event_type\":\"heartbeat\",\"timestamp_ms\":%llu}",
        DOORBELL_ID, (unsigned long long)ts_ms);

    sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
    close(sock);
}

static void heartbeat_task(void *arg)
{
    while (!g_wifi_up) { vTaskDelay(pdMS_TO_TICKS(500)); }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
        send_heartbeat();
    }
}

/*---------------------------------------------------------------------------*/
/* app_main                                                                    */
/*---------------------------------------------------------------------------*/

void app_main(void)
{
    nvs_flash_init();

    g_trigger_sem = xSemaphoreCreateCounting(5, 0);

    if (ESP_OK != camera_init())
    {
        ESP_LOGE(TAG, "Camera init failed, halting");
        return;
    }

    button_init();
    wifi_init();

    xTaskCreate(heartbeat_task, "heartbeat", 4096, NULL, 3, NULL);
    xTaskCreate(capture_task, "capture", 8192, NULL, 5, NULL);
}
