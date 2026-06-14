/******************************************************************************
 * \file main.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief ESP32 doorbell CAM — button-triggered JPEG capture and push to BBB.
 *
 * \details Board: AI-Thinker ESP32-CAM (OV3660), 8MB external PSRAM.
 *          Pinout from AI-Thinker schematic.
 *
 *          Flow (DOORBELL_MODE_SNAPSHOT, default):
 *          boot -> wifi up -> wait for GPIO13 button press
 *          button press -> generate event_id
 *                       -> send UDP event to hub (Lane A, authoritative)
 *                       -> capture JPEG
 *                       -> connect to BBB:9091 -> send header+JPEG -> close
 *
 *          Flow (DOORBELL_MODE_STREAM):
 *          boot -> wifi up -> wait for GPIO13 button press
 *          button press -> generate event_id
 *                       -> send UDP event to hub (Lane A, still fires)
 *                       -> toggle stream state
 *                          ON:  connect to BBB:9093, send device_id byte,
 *                               then loop sending [len:4][jpeg] frames
 *                          OFF: close connection, stop loop
 *
 *          TCP connection is opened per-trigger and closed after send
 *          (SNAPSHOT), or held open for the duration of the stream (STREAM).
 *          Button is debounced in software (DEBOUNCE_MS).
 *          GPIO13 pullup disabled — external LED+resistor acts as pullup.
 *
 * \note    Separate from S3 PIR cam which streams MJPEG to BBB:9090.
 *          Doorbell sends single JPEG to BBB:9091 on each button press
 *          (SNAPSHOT mode) or continuous frames to BBB:9093 (STREAM mode).
 *          Up to MAX_DOORBELL_CAMS (4) devices share each port.
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
 *
 * \note    Trinity integration (2026-06-11):
 *          trinity_wdt / canary / panic / nvs / stats added.
 *          capture_task: blocking xSemaphoreTake(portMAX_DELAY) replaced
 *          with bounded 2s wait so the WDT can be kicked while idle.
 *          heartbeat_task: 30s vTaskDelay chunked into 2s kicks.
 *          TRINITY_CHIP_ESP32_DOORBELL_IDF / "doorbell_log" namespace.
 *
 * \note    Camera quality tuning (2026-06-14):
 *          Sensor confirmed as OV3660, not OV2640 as originally assumed.
 *          OV2640-specific tuning calls (set_gainceiling, set_aec2, set_bpc,
 *          set_wpc, set_sharpness) were misconfiguring the OV3660, producing
 *          near-continuous NO-SOI corrupt frames during idle and streaming.
 *          Removed entirely. jpeg_quality bumped from 6 to 10: smaller frames
 *          (~5-7KB vs ~13KB) reduce WiFi transmission time, lower lag in VLC,
 *          and reduce FB-OVF frequency. VGA at quality 10 is still adequate
 *          resolution for a doorbell cam.
 *
 * \note    Send-loop WDT fix (2026-06-13):
 *          Bumping to VGA increased JPEG size from ~3KB to ~11KB.
 *          send_jpeg_to_bbb()'s send loop had no WDT kicks; under network
 *          congestion this blocked long enough to trip the 5s task_wdt
 *          and abort/reboot mid-send (observed: capture task_wdt abort
 *          ~2.5s after BBB connect, before send completed).
 *
 *          Fix: trinity_wdt_kick() added inside the send loop in
 *          send_jpeg_to_bbb(), after each send() call. Same fix was
 *          already applied to the S3 PIR cam's send_jpeg_to_bbb() when
 *          it was bumped to VGA — should have been carried over here at
 *          the same time but was missed.
 *
 * \note    WiFi power-save disabled (2026-06-13):
 *          Diagnostic send-loop logging showed send() throughput of
 *          ~5740 bytes per ~1.5s call (~30 kbps), then EAGAIN under a
 *          1s SO_SNDTIMEO — far below normal WiFi TX speed. Root cause:
 *          default modem-sleep power-save (WIFI_PS_MIN_MODEM) puts the
 *          radio to sleep between DTIM beacons, badly throttling TCP
 *          throughput right after connect.
 *
 *          Fix: esp_wifi_set_ps(WIFI_PS_NONE) added in wifi_init(),
 *          matching the S3 PIR cam (which already had this fix from its
 *          own frame-capture deadlock investigation on 2026-06-12).
 *
 * \note    Passive buzzer (2026-06-13):
 *          Passive buzzer added on GPIO14 via 2N7000 N-channel MOSFET.
 *          buzzer_beep() called on button press for immediate audio
 *          feedback before capture and send. LEDC_TIMER_1/CHANNEL_1
 *          reserved for buzzer; camera uses LEDC_TIMER_0/CHANNEL_0.
 *
 * \note    DOORBELL_MODE build flag (2026-06-14):
 *          Added DOORBELL_MODE_SNAPSHOT (default) and DOORBELL_MODE_STREAM
 *          build-time modes, allowing mixed deployment across the 4
 *          doorbell cams. SNAPSHOT path is unchanged from prior behaviour.
 *          STREAM path adds stream_task(): button press toggles a
 *          persistent TCP connection to BBB:9093 carrying continuous
 *          [len:4][jpeg] frames, relayed by doorbell_stream_daemon ->
 *          ffmpeg -> mediamtx RTSP. Lane A UDP event still fires on every
 *          button press in both modes. buzzer_beep() called on stream
 *          toggle ON and toggle OFF for audio feedback. No inference is
 *          performed in the stream path — pure video transport.
 *
 * \note    Camera DMA buffer tuning (2026-06-14):
 *          fb_count bumped 1 -> 2, grab_mode changed CAMERA_GRAB_WHEN_EMPTY
 *          -> CAMERA_GRAB_LATEST. Under STREAM mode, network congestion
 *          caused the single DMA buffer to overflow (FB-OVF) while
 *          stream_send_all() was blocked retrying a slow send. The second
 *          buffer gives the DMA engine a slot to write into during that
 *          window; GRAB_LATEST ensures the driver continuously overwrites
 *          the oldest buffer so stream_task always dequeues the freshest
 *          frame rather than accumulating lag. SNAPSHOT mode is unaffected:
 *          the camera is idle between presses so no overflow is possible,
 *          and GRAB_LATEST on a rapid double-press delivers the most recent
 *          image of the visitor rather than queuing historical frames.
 *          Both buffers allocated in PSRAM (fb_location unchanged).
 *
 * \note    Truncation fix (2026-06-14):
 *          Root cause: stream_send_all() treated SO_SNDTIMEO-induced send()
 *          failures as fatal socket errors, closing the connection mid-frame.
 *          On lwIP/ESP-IDF, SO_SNDTIMEO expiry returns send() == -1 with
 *          errno == ETIMEDOUT (not always EAGAIN/EWOULDBLOCK). The original
 *          retry condition only checked EAGAIN/EWOULDBLOCK, so ETIMEDOUT
 *          fell through to "return false", causing the higher layer to close
 *          the socket. The BBB saw the resulting TCP FIN mid-frame and logged
 *          "JPEG recv truncated" — a false corruption report triggered by a
 *          soft send stall, not actual data loss.
 *
 *          Fix: ETIMEDOUT added to the transient-retry set in
 *          stream_send_all(). errno is only evaluated when send() returns
 *          < 0, since lwIP only guarantees errno validity on failure.
 *          n == 0 (shouldn't occur for non-zero sends on lwIP but handled
 *          defensively) is treated as retry-safe rather than fatal.
 *
 *          Counterpart BBB fix in doorbell_stream_daemon.c: recv_all()
 *          now distinguishes clean TCP FIN (return -2) from transport
 *          error (return -1) so the log reflects the true cause.
 ******************************************************************************/

#include <string.h>
#include <stdio.h>
#include <errno.h>
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
#include "trinity_log.h"
#include "buzzer.h"

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
        .frame_size     = FRAMESIZE_VGA,
        .jpeg_quality   = 10,              /* quality 10: ~5-7KB per VGA frame,
                                            * good balance of speed and clarity
                                            * for a doorbell cam over WiFi.
                                            * Was 6 (~13KB) — tuned down after
                                            * confirming sensor is OV3660, not
                                            * OV2640 as originally assumed.    */
        .fb_count       = 2,                  /* was 1 — absorbs network jitter
                                               * in STREAM mode; both bufs in
                                               * PSRAM (fb_location below).  */
        .fb_location    = CAMERA_FB_IN_PSRAM,
        .grab_mode      = CAMERA_GRAB_LATEST, /* was CAMERA_GRAB_WHEN_EMPTY —
                                               * driver overwrites oldest buf
                                               * so stream_task always gets
                                               * the freshest frame.         */
    };

    esp_err_t err = esp_camera_init(&config);
    if (ESP_OK != err)
    {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    /* Sensor tuning removed (2026-06-14): the tuning block previously here
     * (set_gainceiling, set_aec2, set_bpc, set_wpc, set_sharpness) was
     * written for OV2640 and was misconfiguring the actual sensor (OV3660),
     * producing near-continuous NO-SOI corrupt frames. Removed entirely;
     * OV3660 defaults produce clean frames without manual register tuning. */

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

    esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ESP_OK != ps_err)
    {
        ESP_LOGW(TAG, "esp_wifi_set_ps(WIFI_PS_NONE) failed: 0x%x", ps_err);
    }
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

#if DOORBELL_MODE == DOORBELL_MODE_SNAPSHOT

/*---------------------------------------------------------------------------*/
/* Lane B — TCP JPEG to BBB (payload, best-effort) — SNAPSHOT mode            */
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
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(g_tcp_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

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

    int hn = send(g_tcp_sock, hdr_buf, CAM_HEADER_SIZE, 0);
    ESP_LOGI(TAG, "Header send() returned %d (errno=%d %s)",
             hn, errno, strerror(errno));
    if (CAM_HEADER_SIZE != hn)
    {
        return false;
    }

    size_t sent  = 0;
    int    iters = 0;
    while (sent < p_fb->len)
    {
        int n = send(g_tcp_sock, p_fb->buf + sent, p_fb->len - sent, 0);
        ESP_LOGI(TAG, "  send() iter=%d req=%zu ret=%d errno=%d (%s) sent_so_far=%zu/%zu",
                 iters, p_fb->len - sent, n, errno, strerror(errno),
                 sent, p_fb->len);
        if (0 >= n)
        {
            ESP_LOGE(TAG, "send() failed/zero at sent=%zu/%zu — aborting send",
                     sent, p_fb->len);
            return false;
        }
        sent += (size_t)n;
        iters++;
        trinity_wdt_kick();
    }

    ESP_LOGI(TAG, "Sent JPEG %zu bytes in %d send() calls event_id=%08lx%08lx",
             p_fb->len, iters,
             (unsigned long)(event_id >> 32),
             (unsigned long)(event_id & 0xFFFFFFFFUL));
    return true;
}

#else /* DOORBELL_MODE == DOORBELL_MODE_STREAM */

/*---------------------------------------------------------------------------*/
/* Lane B — TCP MJPEG stream to BBB — STREAM mode                            */
/*---------------------------------------------------------------------------*/

static bool stream_send_all(const void *buf, size_t len);

/**
 * \brief Connect to BBB:9093 and send the one-shot device_id byte.
 *
 * \details Wire protocol (matches doorbell_stream_daemon):
 *          connect -> send [device_id:1] once -> loop sending
 *          [jpeg_len:4 big-endian][jpeg bytes] per frame.
 *
 * \return true on success, false on failure (socket left closed).
 */
static bool stream_tcp_connect_once(void)
{
    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(BBB_STREAM_PORT);
    inet_pton(AF_INET, BBB_HOST, &addr.sin_addr);

    if (g_tcp_sock >= 0) { close(g_tcp_sock); g_tcp_sock = -1; }

    g_tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (0 > g_tcp_sock) { return false; }

    if (0 != connect(g_tcp_sock, (struct sockaddr *)&addr, sizeof(addr)))
    {
        close(g_tcp_sock);
        g_tcp_sock = -1;
        return false;
    }

    /* SO_SNDTIMEO of 500ms — NOT used as a "this send must finish within
     * X" deadline (the link can't sustain ~13KB frames that fast). Instead
     * stream_send_all() treats a 500ms EAGAIN/EWOULDBLOCK/ETIMEDOUT as
     * "still sending, call again", looping with a trinity_wdt_kick() every
     * ~500ms. This decouples total per-frame send time (which can
     * legitimately exceed the 5s WDT period under WiFi congestion) from
     * the WDT, while still kicking often enough that no single iteration
     * can starve it.
     *
     * History: 1s and 2s timeouts both caused "frame send() failed" every
     * ~5-6s because the link genuinely takes longer than that per ~13KB
     * frame; a 5s timeout tripped the WDT directly since a single blocked
     * send() could consume the whole WDT period with no intervening kick.
     * 500ms + retry-loop fixes both. ETIMEDOUT (not just EAGAIN) must be
     * in the retry set — lwIP maps SO_SNDTIMEO expiry to ETIMEDOUT, not
     * always EAGAIN. Missing this caused the original truncation bug. */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 500000 };
    setsockopt(g_tcp_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    uint8_t device_id_byte = (uint8_t)DOORBELL_ID;
    if (!stream_send_all(&device_id_byte, 1))
    {
        ESP_LOGW(TAG, "Stream: device_id byte send() failed (errno=%d %s)",
                 errno, strerror(errno));
        close(g_tcp_sock);
        g_tcp_sock = -1;
        return false;
    }

    ESP_LOGI(TAG, "Stream: connected to BBB %s:%d (device_id=%d)",
             BBB_HOST, BBB_STREAM_PORT, DOORBELL_ID);
    return true;
}

/**
 * \brief Send exactly len bytes on g_tcp_sock, tolerating SO_SNDTIMEO
 *        timeouts on a slow link by retrying, kicking the Trinity WDT
 *        every retry so total send time can exceed the WDT period without
 *        tripping it.
 *
 * \details g_tcp_sock has SO_SNDTIMEO set to 500ms. On lwIP/ESP-IDF,
 *          SO_SNDTIMEO expiry returns send() == -1 with errno == ETIMEDOUT
 *          (and sometimes EAGAIN or EWOULDBLOCK). All three are treated as
 *          transient backpressure — "keep trying", not failure.
 *
 *          errno is only evaluated when send() returns < 0. lwIP only
 *          guarantees errno validity on failure; reading it after n > 0 or
 *          n == 0 would read stale state.
 *
 *          n == 0 should not occur for non-zero sends on lwIP but is
 *          treated as retry-safe rather than fatal as a defensive measure.
 *
 *          Any other negative errno (ECONNRESET, EPIPE, ENOTCONN, ...)
 *          is a real connection failure and returns false immediately.
 *
 * \param[in] buf  Data to send.
 * \param[in] len  Number of bytes to send.
 * \return         true if all len bytes were sent, false on hard failure.
 */
static bool stream_send_all(const void *buf, size_t len)
{
    const uint8_t *p    = (const uint8_t *)buf;
    size_t          sent = 0;

    while (sent < len)
    {
        trinity_wdt_kick();

        int n = send(g_tcp_sock, p + sent, len - sent, 0);

        if (n > 0)
        {
            sent += (size_t)n;
            continue;
        }

        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ETIMEDOUT)
            {
                /* Transient: SO_SNDTIMEO expired or send buffer full.
                 * lwIP maps SO_SNDTIMEO to ETIMEDOUT; EAGAIN/EWOULDBLOCK
                 * can also appear. Loop and retry — not a dead socket. */
                continue;
            }

            /* Any other errno is a real connection failure. */
            return false;
        }

        /* n == 0: shouldn't happen for non-zero sends on lwIP, but treat
         * as retry-safe rather than fatal. */
    }

    return true;
}

/**
 * \brief Send one [len:4][jpeg] frame on the already-connected stream socket.
 *
 * \param[in] p_fb  Camera frame buffer to send.
 * \return          true if the full frame was sent, false on any failure.
 */
static bool stream_send_frame(camera_fb_t *p_fb)
{
    uint8_t len_hdr[4];
    cam_pack_stream_frame_hdr(len_hdr, (uint32_t)p_fb->len);

    if (!stream_send_all(len_hdr, sizeof(len_hdr)))
    {
        ESP_LOGW(TAG, "Stream: frame length header send() failed (errno=%d %s)",
                 errno, strerror(errno));
        return false;
    }

    if (!stream_send_all(p_fb->buf, p_fb->len))
    {
        ESP_LOGE(TAG, "Stream: frame payload send() failed (errno=%d %s)",
                 errno, strerror(errno));
        return false;
    }

    return true;
}

#endif /* DOORBELL_MODE */

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
/* Common: wait for a debounced button press, fire Lane A UDP event           */
/*---------------------------------------------------------------------------*/

/**
 * \brief Block (with WDT-friendly bounded waits) until a debounced button
 *        press occurs, then fire the Lane A UDP event.
 *
 * \details Shared by capture_task (SNAPSHOT) and stream_task (STREAM).
 *          Returns the generated event_id, which both modes include for
 *          correlation even though the STREAM path currently has no Lane B
 *          per-frame event_id field.
 *
 * \return event_id for this press.
 */
static uint64_t wait_for_button_press(void)
{
    while (1)
    {
        if (pdTRUE != xSemaphoreTake(g_trigger_sem, pdMS_TO_TICKS(2000)))
        {
            trinity_wdt_kick();
            continue;
        }
        trinity_wdt_kick();
        break;
    }

    /* Debounce */
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));

    /* Drain bounce */
    while (xSemaphoreTake(g_trigger_sem, 0) == pdTRUE) {}

    ESP_LOGI(TAG, "Doorbell pressed — generating event");

    uint64_t event_id = cam_make_event_id(g_mac_tail, g_boot_ms, g_event_seq++);

    /* Lane A — fires on every button press in both modes */
    send_udp_event(event_id);

    return event_id;
}

#if DOORBELL_MODE == DOORBELL_MODE_SNAPSHOT

/*---------------------------------------------------------------------------*/
/* Capture task — SNAPSHOT mode                                                */
/*---------------------------------------------------------------------------*/

static void capture_task(void *arg)
{
    while (!g_wifi_up) { vTaskDelay(pdMS_TO_TICKS(500)); }

    ESP_LOGI(TAG, "Ready — waiting for doorbell button press (device_id=%d, mode=SNAPSHOT)",
             DOORBELL_ID);

    /* ---- Trinity: register after WiFi is up, before the main loop ---- */
    trinity_wdt_add();

    while (1)
    {
        uint64_t event_id = wait_for_button_press();

        /* Immediate audio feedback on button press — fires before capture
         * and send so the user knows the press registered even if BBB is
         * slow or unreachable. */
        buzzer_beep();

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

#else /* DOORBELL_MODE == DOORBELL_MODE_STREAM */

/*---------------------------------------------------------------------------*/
/* Stream task — STREAM mode                                                   */
/*---------------------------------------------------------------------------*/

/**
 * \brief Button-toggled continuous MJPEG stream to BBB:9093.
 *
 * \details Each button press toggles streaming on/off:
 *          - Toggle ON:  retry-connect to BBB:9093 (3x, RECONNECT_MS apart,
 *                         matching the SNAPSHOT pattern), send device_id
 *                         byte, then begin the frame loop.
 *          - Toggle OFF: close the TCP connection, stop the frame loop.
 *
 *          While streaming, the frame loop captures and sends frames as
 *          fast as the camera/network allow, with a bounded wait on the
 *          trigger semaphore so a button press (toggle OFF) and the WDT
 *          kick are both serviced promptly without a separate poll.
 *
 *          On an unexpected BBB disconnect mid-stream (send failure), the
 *          task retries the connection up to 3x (RECONNECT_MS apart) before
 *          giving up and falling back to the OFF state, matching the
 *          existing SNAPSHOT reconnect pattern.
 *
 *          Lane A UDP event fires on every press (toggle ON and toggle OFF),
 *          via wait_for_button_press(). No inference is performed; this is
 *          pure video transport.
 */
static void stream_task(void *arg)
{
    while (!g_wifi_up) { vTaskDelay(pdMS_TO_TICKS(500)); }

    ESP_LOGI(TAG, "Ready — waiting for doorbell button press (device_id=%d, mode=STREAM)",
             DOORBELL_ID);

    /* ---- Trinity: register after WiFi is up, before the main loop ---- */
    trinity_wdt_add();

    bool streaming = false;

    while (1)
    {
        if (!streaming)
        {
            /* Idle: wait for a press to toggle streaming ON */
            (void)wait_for_button_press();

            buzzer_beep();   /* toggle ON feedback */

            bool connected = false;
            for (int i = 0; i < 3; i++)
            {
                if (stream_tcp_connect_once()) { connected = true; break; }
                ESP_LOGW(TAG, "Stream connect attempt %d failed", i + 1);
                vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
            }

            if (!connected)
            {
                ESP_LOGE(TAG, "Stream: BBB unreachable, staying OFF");
                continue;
            }

            ESP_LOGI(TAG, "Stream: started (device_id=%d)", DOORBELL_ID);
            streaming = true;
            continue;
        }

        /* Streaming: send frames until a press toggles OFF or the
         * connection drops. Bounded semaphore wait services both the
         * toggle-OFF press and the WDT kick while we're not actively
         * blocked in send(). */
        if (pdTRUE == xSemaphoreTake(g_trigger_sem, 0))
        {
            /* Debounce + drain, matching wait_for_button_press() */
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            while (xSemaphoreTake(g_trigger_sem, 0) == pdTRUE) {}

            ESP_LOGI(TAG, "Doorbell pressed — generating event");
            uint64_t event_id = cam_make_event_id(g_mac_tail, g_boot_ms, g_event_seq++);
            send_udp_event(event_id);   /* Lane A — toggle OFF press */

            buzzer_beep();   /* toggle OFF feedback */

            close(g_tcp_sock);
            g_tcp_sock = -1;
            streaming = false;
            ESP_LOGI(TAG, "Stream: stopped (device_id=%d)", DOORBELL_ID);
            continue;
        }

        trinity_wdt_kick();

        camera_fb_t *p_fb = esp_camera_fb_get();
        if (NULL == p_fb)
        {
            ESP_LOGW(TAG, "Stream: capture failed, skipping frame");
            continue;
        }

        bool ok = stream_send_frame(p_fb);
        esp_camera_fb_return(p_fb);

        if (!ok)
        {
            ESP_LOGW(TAG, "Stream: frame send failed, attempting reconnect");
            close(g_tcp_sock);
            g_tcp_sock = -1;

            bool reconnected = false;
            for (int i = 0; i < 3; i++)
            {
                trinity_wdt_kick();
                if (stream_tcp_connect_once()) { reconnected = true; break; }
                ESP_LOGW(TAG, "Stream reconnect attempt %d failed", i + 1);
                vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
            }

            if (!reconnected)
            {
                ESP_LOGE(TAG, "Stream: BBB unreachable after retries, stopping stream");
                streaming = false;
            }
        }
    }
}

#endif /* DOORBELL_MODE */

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

    /* ---- Trinity: register before entering the heartbeat loop ---- */
    trinity_wdt_add();

    while (1)
    {
        /* ---- Trinity: chunk the 30s interval into 2s kicks so the WDT
         *      (5s timeout) never starves during the wait.            ---- */
        for (int elapsed = 0; elapsed < HEARTBEAT_INTERVAL_MS; elapsed += 2000)
        {
            vTaskDelay(pdMS_TO_TICKS(2000));
            trinity_wdt_kick();
        }
        send_heartbeat();
    }
}

/*---------------------------------------------------------------------------*/
/* app_main                                                                    */
/*---------------------------------------------------------------------------*/

void app_main(void)
{
    nvs_flash_init();

    /* ---- Trinity: dump previous fault record, init logging + WDT ---- */
    trinity_log_dump_previous();
    trinity_log_init();
    trinity_wdt_init();

    g_trigger_sem = xSemaphoreCreateCounting(5, 0);

    if (ESP_OK != camera_init())
    {
        ESP_LOGE(TAG, "Camera init failed, halting");
        return;
    }

    button_init();
    buzzer_init();
    wifi_init();

    xTaskCreate(heartbeat_task, "heartbeat", 4096, NULL, 3, NULL);

#if DOORBELL_MODE == DOORBELL_MODE_SNAPSHOT
    /* Stack bumped 8192 -> 12288: VGA JPEG buffers (~10KB) plus the added
     * sensor tuning calls in camera_init() increase capture_task's stack
     * footprint versus the original QQVGA/quality-12 configuration. */
    xTaskCreate(capture_task, "capture", 12288, NULL, 5, NULL);
#else
    /* stream_task carries the same per-frame buffers as capture_task, so
     * use the same 12288-byte stack sizing. */
    xTaskCreate(stream_task, "stream", 12288, NULL, 5, NULL);
#endif

    /* -----------------------------------------------------------------------
     * log main-task stack high-water-mark before app_main() exits.
     *
     * trinity_wdt_kick() covers all spawned Trinity tasks, but app_main()
     * itself never calls it.  Sample here so a shrinking margin is visible
     * in the serial log on every boot AND survives a crash via the NVS fault
     * log (trinity_log_dump_previous() on next boot).
     * --------------------------------------------------------------------- */
    {
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG,
                 "[TRINITY] app_main exit stack HWM: %u words (%u B)",
                 (unsigned)hwm,
                 (unsigned)(hwm * sizeof(StackType_t)));

#ifndef CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS
#define CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS 256u
#endif
        if (hwm < (UBaseType_t)CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS)
        {
            ESP_LOGW(TAG,
                     "[TRINITY] LOW STACK on main task at exit: hwm=%u words threshold=%u words",
                     (unsigned)hwm,
                     (unsigned)CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS);
            trinity_log_record_low_stack((uint32_t)hwm);
        }
    }
}
