/******************************************************************************
 * \file main.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief ESP32-S3-CAM PIR-triggered JPEG capture and push to BeagleBone.
 *
 * \details Board: ESP32-S3-CAM (OV2640/OV3660).
 *          Pinout from board silkscreen diagram.
 *
 *          Flow:
 *          boot -> wifi up -> listen for UDP "CAPTURE" on UDP_TRIGGER_PORT
 *          trigger received -> connect to BBB -> stream frames for CAM_CLIP_DURATION_MS -> close
 *
 *          TCP connection is opened per-trigger and closed after send.
 *          No persistent connection; BBB never times out waiting for data.
 *
 * \note    Initial version: persistent TCP connect at boot, triggered send.
 *          BBB recv timeout caused connection loss before trigger fired.
 *
 * \note    Connect-per-trigger fix (2026-05-31):
 *          tcp_connect() moved inside capture loop, called after semaphore
 *          take and successful fb_get. Socket closed after each send.
 *          Eliminates BBB-side timeout on long inter-trigger intervals.
 *
 * \note    Resolution bump (2026-05-31):
 *          FRAMESIZE_QQVGA (160x120) -> FRAMESIZE_QVGA (320x240).
 *          160x120 upscaled to model input 300x300 produced 0.00 confidence.
 *
 * \note    XCLK bump (2026-05-31):
 *          10 MHz -> 20 MHz for better OV2640 frame quality.
 *
 * \note    Clip streaming (2026-06-11):
 *          capture_task changed from single JPEG to 10s clip stream.
 *          On trigger: connect to BBB, send frames every CAM_CLIP_FRAME_MS
 *          for CAM_CLIP_DURATION_MS ms, then close. BBB detects end of clip
 *          by connection close. Wire protocol unchanged: [len:4][jpeg] per frame.
 *
 * \note    Trinity integration (2026-06-11):
 *          trinity_wdt / canary / panic / nvs / stats added.
 *          udp_trigger_task: recv() given a 2s SO_RCVTIMEO so the loop can
 *          kick the WDT while idle.
 *          capture_task: blocking xSemaphoreTake(portMAX_DELAY) replaced
 *          with bounded 2s wait so the WDT can be kicked while idle.
 *          heartbeat_task: CAM_HEARTBEAT_MS + jitter delay chunked into 2s
 *          kicks (WDT timeout is 5s).
 *          TRINITY_CHIP_ESP32_CAM_IDF / "cam_log" namespace.
 *
 * \note    Main-task stack overflow fix (2026-06-12):
 *          Root cause of the post-wifi_init() CORRUPT HEAP / Bad tail crash
 *          was main task stack overflow, not heap corruption from
 *          malloc/free. Fixed via:
 *              CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192
 *          Confirmed by clean boot: WiFi connects, IP obtained, heartbeat
 *          task sends successfully with no crash. Debug heap checkpoints
 *          removed.
 ******************************************************************************/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_camera.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "wifi_secrets.h"
#include "cam_logic.h"
#include "trinity_log.h"

static const char *TAG = "CAM";

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
        .frame_size     = FRAMESIZE_QVGA,
        .jpeg_quality   = 12,
        .fb_count       = 1,
        .fb_location    = CAMERA_FB_IN_PSRAM,
        .grab_mode      = CAMERA_GRAB_WHEN_EMPTY,
    };

    esp_err_t err = esp_camera_init(&config);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(500)); /* warm up */
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
/* TCP to BBB                                                                  */
/*---------------------------------------------------------------------------*/

static void tcp_connect(void)
{
    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(BBB_PORT);
    inet_pton(AF_INET, BBB_HOST, &addr.sin_addr);

    while (1)
    {
        if (g_tcp_sock >= 0) { close(g_tcp_sock); g_tcp_sock = -1; }

        g_tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (0 > g_tcp_sock)
        {
            ESP_LOGE(TAG, "socket() failed");
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
            continue;
        }

        if (0 == connect(g_tcp_sock, (struct sockaddr *)&addr, sizeof(addr)))
        {
            ESP_LOGI(TAG, "Connected to BBB %s:%d", BBB_HOST, BBB_PORT);
            return;
        }

        ESP_LOGW(TAG, "Connect to BBB failed, retrying...");
        close(g_tcp_sock);
        g_tcp_sock = -1;
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
    }
}

static bool send_jpeg_to_bbb(camera_fb_t *p_fb)
{
    uint8_t  hdr[4];
    uint32_t len = (uint32_t)p_fb->len;

    cam_pack_jpeg_header(hdr, len);

    if (4 != send(g_tcp_sock, hdr, 4, 0)) { return false; }

    size_t sent = 0;
    while (sent < p_fb->len)
    {
        int n = send(g_tcp_sock, p_fb->buf + sent, p_fb->len - sent, 0);
        if (0 >= n) { return false; }
        sent += (size_t)n;
    }

    ESP_LOGI(TAG, "Sent JPEG %zu bytes to BBB", p_fb->len);
    return true;
}

/*---------------------------------------------------------------------------*/
/* UDP trigger task                                                            */
/*---------------------------------------------------------------------------*/

static void udp_trigger_task(void *arg)
{
    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(UDP_TRIGGER_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    /* ---- Trinity: bounded recv so the loop can kick the WDT while idle ---- */
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    trinity_wdt_add();

    char buf[32];
    while (1)
    {
        int n = recv(sock, buf, sizeof(buf) - 1, 0);

        trinity_wdt_kick();

        if (n > 0)
        {
            buf[n] = '\0';
            if (cam_is_trigger(buf))
            {
                ESP_LOGI(TAG, "UDP trigger received");
                xSemaphoreGive(g_trigger_sem);
            }
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Capture task                                                                */
/*---------------------------------------------------------------------------*/

static void capture_task(void *arg)
{
    while (!g_wifi_up) { vTaskDelay(pdMS_TO_TICKS(500)); }

    /* ---- Trinity: register after WiFi is up, before the main loop ---- */
    trinity_wdt_add();

    while (1)
    {
        /* ---- Trinity: bounded wait so the loop can kick the WDT while
         *      idle, waiting for a trigger.                            ---- */
        if (pdTRUE != xSemaphoreTake(g_trigger_sem, pdMS_TO_TICKS(2000)))
        {
            trinity_wdt_kick();
            continue;
        }
        trinity_wdt_kick();

        ESP_LOGI(TAG, "Triggered — starting clip");

        tcp_connect();
        trinity_wdt_kick();

        uint32_t elapsed = 0;
        while (elapsed < CAM_CLIP_DURATION_MS)
        {
            camera_fb_t *p_fb = esp_camera_fb_get();
            if (NULL != p_fb)
            {
                if (!send_jpeg_to_bbb(p_fb))
                {
                    ESP_LOGW(TAG, "Send failed — aborting clip");
                    esp_camera_fb_return(p_fb);
                    break;
                }
                esp_camera_fb_return(p_fb);
            }
            else
            {
                ESP_LOGW(TAG, "Capture failed at %lu ms", (unsigned long)elapsed);
            }

            vTaskDelay(pdMS_TO_TICKS(CAM_CLIP_FRAME_MS));
            trinity_wdt_kick();
            elapsed += CAM_CLIP_FRAME_MS;
        }

        ESP_LOGI(TAG, "Clip complete — %lu ms", (unsigned long)elapsed);
        close(g_tcp_sock);
        g_tcp_sock = -1;
    }
}

/*---------------------------------------------------------------------------*/
/* Heartbeat task                                                              */
/*---------------------------------------------------------------------------*/

static void heartbeat_task(void *arg)
{
    struct sockaddr_in addr = {0};
    int      sock           = -1;
    char     buf[128];
    uint32_t seq            = 0;

    (void)arg;

    while (!g_wifi_up) { vTaskDelay(pdMS_TO_TICKS(500)); }

    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(HUB_HEARTBEAT_PORT);
    inet_pton(AF_INET, HUB_HOST, &addr.sin_addr);

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "heartbeat socket() failed");
        vTaskDelete(NULL);
        return;
    }

    /* ---- Trinity: register before entering the heartbeat loop ---- */
    trinity_wdt_add();

    while (1)
    {
        snprintf(buf, sizeof(buf),
                 "{\"device_id\":%d,\"device_type\":\"cam\","
                 "\"event_type\":\"heartbeat\",\"seq\":%lu,"
                 "\"timestamp_ms\":%lu}",
                 (int)CAM_SLOT,
                 (unsigned long)seq++,
                 (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS));

        sendto(sock, buf, strlen(buf), 0,
               (struct sockaddr *)&addr, sizeof(addr));

        ESP_LOGI(TAG, "Heartbeat sent seq=%lu", (unsigned long)(seq - 1));

        /* ---- Trinity: chunk the heartbeat interval into 2s kicks so the
         *      WDT (5s timeout) never starves during the wait.        ---- */
        uint32_t jitter_ms = esp_random() % CAM_HEARTBEAT_JITTER;
        uint32_t total_ms  = CAM_HEARTBEAT_MS + jitter_ms;

        for (uint32_t elapsed = 0; elapsed < total_ms; elapsed += 2000)
        {
            uint32_t chunk = (total_ms - elapsed > 2000) ? 2000 : (total_ms - elapsed);
            vTaskDelay(pdMS_TO_TICKS(chunk));
            trinity_wdt_kick();
        }
    }
}

/*---------------------------------------------------------------------------*/
/* app_main                                                                    */
/*---------------------------------------------------------------------------*/

void app_main(void)
{
    nvs_flash_init();

    trinity_log_dump_previous();
    trinity_log_init();
    trinity_wdt_init();

    g_trigger_sem = xSemaphoreCreateCounting(5, 0);

    if (ESP_OK != camera_init())
    {
        ESP_LOGE(TAG, "Camera init failed, halting");
        return;
    }

    wifi_init();

    xTaskCreate(udp_trigger_task, "udp_trigger", 4096, NULL, 5, NULL);
    xTaskCreate(capture_task,     "capture",     8192, NULL, 5, NULL);
    xTaskCreate(heartbeat_task,   "heartbeat",   4096, NULL, 4, NULL);
}
