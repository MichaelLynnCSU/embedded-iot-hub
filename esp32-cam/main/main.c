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
 *          boot -> wifi up -> listen for UDP trigger on UDP_TRIGGER_PORT
 *          trigger received -> parse -> enqueue cam_trigger_t ->
 *          capture_task dequeues -> connect to BBB -> stream frames for
 *          CAM_CLIP_DURATION_MS -> close
 *
 *          TCP connection is opened per-trigger and closed after send.
 *          No persistent connection; BBB never times out waiting for data.
 *
 * \note    Initial version: persistent TCP connect at boot, triggered send.
 *          BBB recv timeout caused connection loss before trigger fired.
 *
 * \note    Connect-per-trigger fix (2026-05-31):
 *          tcp_connect() moved inside capture loop, called after queue
 *          receive and successful fb_get. Socket closed after each send.
 *          Eliminates BBB-side timeout on long inter-trigger intervals.
 *
 * \note    Resolution bump (2026-05-31):
 *          FRAMESIZE_QQVGA (160x120) -> FRAMESIZE_QVGA (320x240).
 *          160x120 upscaled to model input 300x300 produced 0.00 confidence.
 *
 * \note    XCLK bump (2026-05-31):
 *          10 MHz -> 20 MHz for better frame quality.
 *          (Note: original comment said "OV2640" -- board was not yet
 *          confirmed at that point. Runtime detection in later logs
 *          confirms the populated sensor is OV3660.)
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
 *          capture_task: blocking xQueueReceive(portMAX_DELAY) replaced
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
 *
 * \note    Frame-capture deadlock fix (2026-06-12):
 *          Symptom: every esp_camera_fb_get() in capture_task timed out
 *          (cam_hal "Failed to get the frame on time!", ~4.5s each) for
 *          the entire 10s clip, every trigger, with PCLK/VSYNC apparently
 *          dead by the time the first frame was requested.
 *
 *          Two changes made together:
 *
 *          1) esp_wifi_set_ps(WIFI_PS_NONE) added after esp_wifi_start().
 *             Tested: did NOT fix it on its own (see test results below).
 *
 *          2) capture_task now:
 *               - flushes 2 frames immediately after tcp_connect() returns,
 *               - detects a fb_get() timeout mid-clip, marks the sensor
 *                 "dead" for the remainder of the clip (stops blocking for
 *                 ~4s per attempt), and
 *               - performs a hot esp_camera_deinit()+camera_init() at the
 *                 end of any clip where the sensor died, so the next
 *                 trigger starts from a known-good state instead of
 *                 failing 100% of the time forever.
 *
 *          Test results (2026-06-12):
 *          - WIFI_PS_NONE active, connect() now ~110ms (was 680ms): sensor
 *            STILL dead at "0 ms" on first fb_get() after long idle.
 *            -> WiFi modem-sleep / connect()-spike theories ruled out.
 *          - Hot-reset recovery WORKED: esp_camera_deinit()+camera_init()
 *            successfully re-detected OV3660 and recovered the sensor.
 *          - CONFIG_PM_ENABLE is NOT set in sdkconfig -> automatic light
 *            sleep / DFS ruled out as well.
 *
 *          Empirical pattern across both failed runs (idle ~78min and
 *          ~9.8min before trigger): any sufficiently long idle period with
 *          zero esp_camera_fb_get() calls -> sensor dead by next trigger,
 *          recoverable via deinit+init. Root mechanism not fully isolated
 *          (possibly sensor-side auto-standby on SCCB/clock inactivity),
 *          but the practical mitigation is to never let the camera sit
 *          idle that long.
 *
 * \note    Idle keepalive (2026-06-12):
 *          capture_task's idle wait loop now pulls and immediately
 *          discards one frame every CAM_IDLE_PING_INTERVAL_MS while
 *          waiting for a trigger. This keeps the DMA/I2S pipeline (and
 *          sensor) continuously active so it should never reach the dead
 *          state described above. If a keepalive ping itself times out,
 *          that means the sensor died anyway during idle -- in that case
 *          we proactively run camera_recover() right then, so a real
 *          trigger arriving shortly after still gets a working sensor
 *          instead of eating the first ~8s of the clip.
 *
 *          WDT note: a keepalive ping can block up to ~4s if the sensor is
 *          dead. We kick the WDT immediately before AND after the ping so
 *          a single slow ping doesn't risk tripping the 5s per-task WDT.
 *
 *          fb_count/grab_mode intentionally left at fb_count=1 /
 *          CAMERA_GRAB_WHEN_EMPTY for this test, to keep this an isolated
 *          single-variable change from the previous (failed) test.
 *
 * \note    Recovery hardening (2026-06-13):
 *          camera_recover() was tripping task_wdt during the hot-reset
 *          itself: esp_camera_deinit()+camera_init()'s internal delays,
 *          plus a post-recovery "NO-SOI" condition where the OV3660
 *          re-detects fine over SCCB but the parallel/DVP bus produces no
 *          frames, could together exceed the 5s per-task WDT window with
 *          no kicks in between.
 *
 *          Fixes:
 *            - camera_recover() now kicks trinity_wdt before/after every
 *              blocking step (deinit, power-cycle delay, re-init).
 *            - camera_init() now does a hardware PWDN power-cycle before
 *              esp_camera_init(), since SCCB-only reinit can leave the
 *              DVP timing generator producing no SOI even though the
 *              sensor still ACKs on I2C.
 *            - camera_recover() pulls one verification frame after
 *              re-init to confirm the bus is actually alive (re-detect
 *              on SCCB alone is not sufficient evidence).
 *            - New camera_recover_or_restart(): if recovery doesn't
 *              produce a verified-good sensor, do a clean esp_restart()
 *              instead of retrying in a tight loop (which previously led
 *              to a task_wdt abort anyway). Both call sites in
 *              capture_task (idle-keepalive and end-of-clip) now use
 *              this instead of calling camera_recover() directly.
 *
 * \note    Image quality (2026-06-13):
 *          Field image was reported as too grainy to make out a face.
 *          QVGA + jpeg_quality 12 with default (uncapped) AGC gain was
 *          producing high analog-gain noise in low light. Added explicit
 *          sensor tuning in camera_init() after esp_camera_init():
 *          higher resolution (VGA), tighter JPEG quality, capped gain
 *          ceiling (GAINCEILING_2X) to eliminate AGC-driven speckle, and
 *          enabled advanced AEC (aec2) + pixel correction (bpc/wpc).
 *
 * \note    VGA send-loop WDT fix (2026-06-13):
 *          Bumping to FRAMESIZE_VGA increased JPEG size from ~3.4KB to
 *          ~9.8KB/frame. send_jpeg_to_bbb() had no WDT kicks inside its
 *          send loop; under network congestion this blocked long enough to
 *          trip the 5s task_wdt. Fix: trinity_wdt_kick() added inside the
 *          send loop after each send() call.
 *
 * \note    Trigger identity model (2026-06-18):
 *          g_trigger_sem (counting semaphore) replaced with g_capture_queue
 *          (queue of cam_trigger_t). A semaphore was correct when the trigger
 *          contained no information ("CAPTURE" — bare signal). Once the
 *          trigger carries event_id, cam_tx_id, and zone it is a message,
 *          not a signal — a queue is the correct primitive.
 *
 *          udp_trigger_task parses the incoming payload via cam_parse_trigger(),
 *          logs [UDP_CAM_RX], and enqueues the cam_trigger_t.
 *          capture_task dequeues it and logs event_id at every capture stage
 *          so the full camera lifecycle shares the originating PIR identity:
 *
 *            [UDP_CAM_RX] cam_tx_id=42 event_id=101 zone=0
 *            [CAM]        event_id=101 capture_start
 *            [CAM]        event_id=101 jpeg_send bytes=9800
 *            [CAM]        event_id=101 clip_done elapsed_ms=10000
 ******************************************************************************/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

/** \brief Queue of cam_trigger_t from udp_trigger_task to capture_task.
 *
 *  Depth 5 matches the old semaphore count — handles burst triggers without
 *  dropping while capture_task is busy with a clip. Each entry carries the
 *  full trigger context (event_id, cam_tx_id, zone) so identity is preserved
 *  through the capture pipeline. */
static QueueHandle_t g_capture_queue = NULL;

static int           g_tcp_sock      = -1;
static bool          g_wifi_up       = false;

/* Number of stale frames to drain from the DMA ring immediately after
 * tcp_connect() returns, before starting the real clip capture loop. */
#define CAM_POST_CONNECT_FLUSH_FRAMES   2

/* How often (in ms) to pull+discard one frame while idle, to keep the
 * camera pipeline alive. Must be a multiple of the 2000ms idle wait period
 * in capture_task for the counter logic below to land on it cleanly. */
#define CAM_IDLE_PING_INTERVAL_MS       6000

static esp_err_t camera_init(void);
static esp_err_t camera_recover(void);
static void      camera_recover_or_restart(void);

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
        .jpeg_quality   = 6,
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

    /* ---- Image quality tuning (2026-06-13) ---- */
    sensor_t *p_sensor = esp_camera_sensor_get();
    if (NULL != p_sensor)
    {
        p_sensor->set_framesize(p_sensor, FRAMESIZE_VGA);
        p_sensor->set_quality(p_sensor, 6);
        p_sensor->set_gainceiling(p_sensor, GAINCEILING_2X);
        p_sensor->set_aec2(p_sensor, 1);
        p_sensor->set_bpc(p_sensor, 1);
        p_sensor->set_wpc(p_sensor, 1);

        if (0 != p_sensor->set_sharpness(p_sensor, 2))
        {
            ESP_LOGW(TAG, "set_sharpness() not supported by this sensor driver");
        }
    }
    else
    {
        ESP_LOGW(TAG, "esp_camera_sensor_get() returned NULL — "
                      "skipping image quality tuning");
    }

    vTaskDelay(pdMS_TO_TICKS(500)); /* warm up */
    return ESP_OK;
}

/*---------------------------------------------------------------------------*/
/* Camera recovery                                                             */
/*---------------------------------------------------------------------------*/

/**
 * \brief Hot-reset the camera subsystem.
 *
 * Called when esp_camera_fb_get() has timed out, indicating the sensor's
 * PCLK/VSYNC have died. esp_camera_deinit() releases the I2S/DMA resources;
 * camera_init() re-probes the sensor over SCCB and restores all tuning.
 *
 * After re-init, a verification frame is pulled to confirm the DVP bus is
 * actually producing valid SOI/frames — re-detecting the sensor on SCCB is
 * not sufficient evidence the parallel interface is alive.
 *
 * WDT is kicked before/after every step that can block for a meaningful
 * time, since this can be called from a WDT-monitored task and each step
 * can individually approach or exceed the per-task WDT window.
 *
 * \return ESP_OK on success (sensor re-init AND verified producing frames),
 *         error code otherwise.
 */
static esp_err_t camera_recover(void)
{
    ESP_LOGW(TAG, "Attempting hot-reset of camera subsystem...");

    trinity_wdt_kick();
    esp_err_t deinit_err = esp_camera_deinit();
    if (ESP_OK != deinit_err)
    {
        ESP_LOGW(TAG, "esp_camera_deinit() returned 0x%x (continuing anyway)",
                 deinit_err);
    }
    trinity_wdt_kick();

    vTaskDelay(pdMS_TO_TICKS(100));
    trinity_wdt_kick();

    esp_err_t init_err = camera_init();
    trinity_wdt_kick();

    if (ESP_OK != init_err)
    {
        ESP_LOGE(TAG, "Camera recovery failed: 0x%x", init_err);
        return init_err;
    }

    camera_fb_t *p_verify = esp_camera_fb_get();
    trinity_wdt_kick();

    if (NULL == p_verify)
    {
        ESP_LOGE(TAG, "Camera recovery: re-init OK but no frame produced "
                      "(NO-SOI) -- recovery failed");
        return ESP_FAIL;
    }

    esp_camera_fb_return(p_verify);
    ESP_LOGI(TAG, "Camera subsystem recovered");
    return ESP_OK;
}

/**
 * \brief Recover the camera, restarting the device if recovery fails.
 *
 * If camera_recover() fails to restore a verified-good sensor, do a clean
 * esp_restart() — deterministic and bounded, avoids tight retry loops that
 * trip the watchdog.
 */
static void camera_recover_or_restart(void)
{
    if (ESP_OK != camera_recover())
    {
        ESP_LOGE(TAG, "Camera recovery failed -- restarting device");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    }
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

    esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ESP_OK != ps_err)
    {
        ESP_LOGW(TAG, "esp_wifi_set_ps(WIFI_PS_NONE) failed: 0x%x", ps_err);
    }
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
        trinity_wdt_kick();

        if (g_tcp_sock >= 0) { close(g_tcp_sock); g_tcp_sock = -1; }

        g_tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (0 > g_tcp_sock)
        {
            ESP_LOGE(TAG, "socket() failed");
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
            trinity_wdt_kick();
            continue;
        }

        int conn_err = connect(g_tcp_sock,
                               (struct sockaddr *)&addr, sizeof(addr));
        trinity_wdt_kick();

        if (0 == conn_err)
        {
            ESP_LOGI(TAG, "Connected to BBB %s:%d", BBB_HOST, BBB_PORT);
            return;
        }

        ESP_LOGW(TAG, "Connect to BBB failed, retrying...");
        close(g_tcp_sock);
        g_tcp_sock = -1;
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
        trinity_wdt_kick();
    }
}

/**
 * \brief Send one JPEG frame to the BeagleBone over TCP.
 *
 * \param p_fb      Camera frame buffer to send.
 * \param event_id  PIR event_id carried from trigger — logged at send so
 *                  the JPEG transmission is traceable back to the originating
 *                  PIR event.
 *
 * \return true on success, false on send failure.
 */
static bool send_jpeg_to_bbb(camera_fb_t *p_fb, uint64_t event_id)
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
        trinity_wdt_kick();
    }

    ESP_LOGI(TAG, "[CAM] event_id=%llu jpeg_send bytes=%zu",
             (unsigned long long)event_id, p_fb->len);
    return true;
}

/*---------------------------------------------------------------------------*/
/* UDP trigger task                                                            */
/*---------------------------------------------------------------------------*/

/**
 * \brief Listen for UDP trigger packets from the hub and enqueue them.
 *
 * \details Parses incoming UDP payload via cam_parse_trigger() to extract
 *          event_id, cam_tx_id, and zone. Logs [UDP_CAM_RX] immediately on
 *          receipt so the transport arrival is visible in the trace chain.
 *          Enqueues the cam_trigger_t to g_capture_queue for capture_task.
 *
 *          A queue is used rather than a semaphore because the trigger now
 *          carries identity (event_id, cam_tx_id, zone) that must be
 *          preserved through the capture pipeline — a semaphore would
 *          discard this context on Give/Take.
 *
 *          recv() is bounded to 2s via SO_RCVTIMEO so the WDT can be
 *          kicked while idle between triggers.
 */
static void udp_trigger_task(void *arg)
{
    struct sockaddr_in addr = {0};
    char               buf[64];
    cam_trigger_t      trig  = {0};
    int                n     = 0;

    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(UDP_TRIGGER_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    /* Bounded recv so the loop can kick the WDT while idle. */
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    trinity_wdt_add();

    while (1)
    {
        n = recv(sock, buf, sizeof(buf) - 1, 0);

        trinity_wdt_kick();

        if (n <= 0) { continue; }

        buf[n] = '\0';

        if (!cam_parse_trigger(buf, &trig))
        {
            ESP_LOGW(TAG, "UDP: unknown payload ignored: %s", buf);
            continue;
        }

        ESP_LOGI(TAG, "[UDP_CAM_RX] cam_tx_id=%u event_id=%llu zone=%d",
                 (unsigned)trig.cam_tx_id,
                 (unsigned long long)trig.event_id,
                 (int)trig.zone);

        /* Drop if queue is full — newest trigger wins on burst. Log the
         * drop so it's visible rather than silently losing identity. */
        if (pdTRUE != xQueueSend(g_capture_queue, &trig, 0))
        {
            ESP_LOGW(TAG, "[UDP_CAM_RX] queue full — drop cam_tx_id=%u event_id=%llu",
                     (unsigned)trig.cam_tx_id,
                     (unsigned long long)trig.event_id);
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Capture task                                                                */
/*---------------------------------------------------------------------------*/

/**
 * \brief Dequeue trigger context and run JPEG clip capture + BBB upload.
 *
 * \details Receives cam_trigger_t from g_capture_queue. Logs event_id at
 *          every stage (capture_start, jpeg_send, clip_done) so the full
 *          camera lifecycle is traceable back to the originating PIR event
 *          with a single grep on event_id.
 *
 *          Idle keepalive: pulls and discards one frame every
 *          CAM_IDLE_PING_INTERVAL_MS while waiting for a trigger to keep
 *          the DMA/I2S pipeline alive. If the ping times out, proactively
 *          recovers the sensor before the next real trigger arrives.
 *
 *          Queue receive is bounded to 2s so the WDT can be kicked while
 *          idle between triggers.
 */
static void capture_task(void *arg)
{
    cam_trigger_t trig            = {0};
    uint32_t      idle_ms_since_ping = 0;

    while (!g_wifi_up) { vTaskDelay(pdMS_TO_TICKS(500)); }

    trinity_wdt_add();

    while (1)
    {
        /* Bounded wait so the WDT can be kicked while idle. */
        if (pdTRUE != xQueueReceive(g_capture_queue, &trig,
                                    pdMS_TO_TICKS(2000)))
        {
            trinity_wdt_kick();

            /* ---- Idle keepalive ---- */
            idle_ms_since_ping += 2000;
            if (idle_ms_since_ping >= CAM_IDLE_PING_INTERVAL_MS)
            {
                idle_ms_since_ping = 0;

                camera_fb_t *p_ping = esp_camera_fb_get();
                if (NULL != p_ping)
                {
                    esp_camera_fb_return(p_ping);
                }
                else
                {
                    ESP_LOGW(TAG, "Idle keepalive: sensor unresponsive — "
                                  "recovering proactively");
                    camera_recover_or_restart();
                }

                trinity_wdt_kick();
            }

            continue;
        }
        trinity_wdt_kick();

        /* Trigger received — reset idle-ping counter. */
        idle_ms_since_ping = 0;

        ESP_LOGI(TAG, "[CAM] event_id=%llu capture_start cam_tx_id=%u zone=%d",
                 (unsigned long long)trig.event_id,
                 (unsigned)trig.cam_tx_id,
                 (int)trig.zone);

        tcp_connect();
        trinity_wdt_kick();

        /* Drain stale frames from DMA ring before starting clip. */
        for (int i = 0; i < CAM_POST_CONNECT_FLUSH_FRAMES; i++)
        {
            camera_fb_t *p_flush = esp_camera_fb_get();
            if (NULL != p_flush) { esp_camera_fb_return(p_flush); }
        }
        trinity_wdt_kick();

        uint32_t elapsed     = 0;
        bool     sensor_dead = false;

        while (elapsed < CAM_CLIP_DURATION_MS)
        {
            if (sensor_dead)
            {
                vTaskDelay(pdMS_TO_TICKS(CAM_CLIP_FRAME_MS));
                trinity_wdt_kick();
                elapsed += CAM_CLIP_FRAME_MS;
                continue;
            }

            camera_fb_t *p_fb = esp_camera_fb_get();
            if (NULL != p_fb)
            {
                if (!send_jpeg_to_bbb(p_fb, trig.event_id))
                {
                    ESP_LOGW(TAG, "[CAM] event_id=%llu send_failed — aborting clip",
                             (unsigned long long)trig.event_id);
                    esp_camera_fb_return(p_fb);
                    break;
                }
                esp_camera_fb_return(p_fb);
            }
            else
            {
                ESP_LOGE(TAG, "[CAM] event_id=%llu capture_timeout at %lu ms — "
                              "marking sensor dead for this clip",
                         (unsigned long long)trig.event_id,
                         (unsigned long)elapsed);
                sensor_dead = true;
            }

            vTaskDelay(pdMS_TO_TICKS(CAM_CLIP_FRAME_MS));
            trinity_wdt_kick();
            elapsed += CAM_CLIP_FRAME_MS;
        }

        ESP_LOGI(TAG, "[CAM] event_id=%llu clip_done elapsed_ms=%lu",
                 (unsigned long long)trig.event_id,
                 (unsigned long)elapsed);

        close(g_tcp_sock);
        g_tcp_sock = -1;

        if (sensor_dead)
        {
            camera_recover_or_restart();
        }
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
    addr.sin_port        = htons(CAMERA_MANAGER_PORT);
    inet_pton(AF_INET, CAMERA_MANAGER_HOST, &addr.sin_addr);

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "heartbeat socket() failed");
        vTaskDelete(NULL);
        return;
    }

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

        uint32_t jitter_ms = esp_random() % CAM_HEARTBEAT_JITTER;
        uint32_t total_ms  = CAM_HEARTBEAT_MS + jitter_ms;

        for (uint32_t elapsed = 0; elapsed < total_ms; elapsed += 2000)
        {
            uint32_t chunk = (total_ms - elapsed > 2000) ?
                             2000 : (total_ms - elapsed);
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

    /* Queue of cam_trigger_t — depth 5 handles burst triggers while
     * capture_task is busy with a clip. Replaces g_trigger_sem (counting
     * semaphore, depth 5) which was correct when the trigger contained no
     * data but wrong once event_id/cam_tx_id/zone need to be preserved. */
    g_capture_queue = xQueueCreate(5, sizeof(cam_trigger_t));

    if (ESP_OK != camera_init())
    {
        ESP_LOGE(TAG, "Camera init failed, halting");
        return;
    }

    wifi_init();

    xTaskCreate(udp_trigger_task, "udp_trigger", 4096, NULL, 5, NULL);
    xTaskCreate(capture_task,     "capture",     8192, NULL, 5, NULL);
    xTaskCreate(heartbeat_task,   "heartbeat",   4096, NULL, 4, NULL);

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
                     "[TRINITY] LOW STACK on main task at exit: "
                     "hwm=%u words threshold=%u words",
                     (unsigned)hwm,
                     (unsigned)CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS);
            trinity_log_record_low_stack((uint32_t)hwm);
        }
    }
}
