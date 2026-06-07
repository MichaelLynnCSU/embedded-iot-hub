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
 *          button press -> capture JPEG -> connect to BBB:9091 -> send -> close
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
 ******************************************************************************/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
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

static SemaphoreHandle_t g_trigger_sem = NULL;
static int               g_tcp_sock    = -1;
static bool              g_wifi_up     = false;

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
/* TCP to BBB — non-blocking, drops frame if unreachable                       */
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

static bool send_jpeg_to_bbb(camera_fb_t *p_fb)
{
    uint8_t hdr_buf[10];
    cam_pack_header(hdr_buf, (uint16_t)DOORBELL_ID, (uint32_t)p_fb->len);

    if (10 != send(g_tcp_sock, hdr_buf, 10, 0)) { return false; }

    size_t sent = 0;
    while (sent < p_fb->len)
    {
        int n = send(g_tcp_sock, p_fb->buf + sent, p_fb->len - sent, 0);
        if (0 >= n) { return false; }
        sent += (size_t)n;
    }

    ESP_LOGI(TAG, "Sent JPEG %zu bytes (device_id=%d)", p_fb->len, DOORBELL_ID);
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

        ESP_LOGI(TAG, "Doorbell pressed — capturing JPEG");

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

        if (!send_jpeg_to_bbb(p_fb))
        {
            ESP_LOGW(TAG, "Send failed");
        }

        esp_camera_fb_return(p_fb);
        close(g_tcp_sock);
        g_tcp_sock = -1;
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

    xTaskCreate(capture_task, "capture", 8192, NULL, 5, NULL);
}
