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
 *          A PWDN power-cycle step was tried as part of this fix but
 *          removed again -- esp_camera_deinit()+camera_init() alone was
 *          sufficient to recover from NO-SOI in testing (see boot log
 *          2026-06-13: "Camera subsystem recovered" + clean 10s clip).
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
 *          Diagnostic framing: if the image is now clean but dark, the
 *          bottleneck is ambient light (add illumination). If it's still
 *          blurry under bright/direct light, the fixed-focus lens on this
 *          board (OV3660, FORIOT ESP32-S3-CAM) is simply mismatched to
 *          the deployment distance -- no software fix for that.
 *
 *          These settings are applied once in camera_init() and persist
 *          across the whole session (idle keepalive + every clip) until
 *          the next camera_recover()/re-init, at which point camera_init()
 *          re-applies them automatically.
 *
 * \note    Known cosmetic issue / TODO (not yet fixed):
 *          Boot log shows:
 *              E (..) gpio: gpio_set_direction(308): GPIO number error
 *              E (..) gpio: gpio_set_level(238): GPIO output gpio_num error
 *          on every camera_init()/camera_recover() call. This is because
 *          CAM_PIN_PWDN and/or CAM_PIN_RESET (defined in cam_logic.h /
 *          board pin header) are set to a non-(-1) value for pins that
 *          aren't actually wired on this board. The esp32-camera driver
 *          then tries to drive those GPIOs and fails. Functionally
 *          harmless (driver still inits/recovers correctly), but should
 *          be cleaned up by setting CAM_PIN_PWDN / CAM_PIN_RESET to -1 in
 *          the pin header if those lines aren't connected.
 *
 * \note    VGA send-loop WDT fix (2026-06-13):
 *          Bumping to FRAMESIZE_VGA (per the image-quality tuning above)
 *          increased JPEG size from ~3.4KB to ~9.8KB/frame. send_jpeg_to_bbb()
 *          had no WDT kicks inside its send loop; under network congestion
 *          this blocked long enough (combined with FB-OVF backpressure from
 *          the camera producing frames faster than they were sent) to trip
 *          the 5s task_wdt and abort/reboot mid-clip.
 *
 *          Fix: trinity_wdt_kick() added inside the send loop in
 *          send_jpeg_to_bbb(), after each send() call.
 *
 *          Remaining tuning TODO: FB-OVF warnings still indicate the
 *          producer (camera DMA) is outpacing the consumer (network send)
 *          at VGA + current CAM_CLIP_FRAME_MS. If this persists, consider
 *          a smaller frame size (e.g. FRAMESIZE_HVGA/CIF) or increasing
 *          CAM_CLIP_FRAME_MS to give more time per frame.
 *
 * \note    XGA diagnostic block removed (2026-06-13):
 *          Root cause identified and diagnostic no longer needed.
 *
 *          The real root problem: dirty sensor state after failed XGA
 *          transition. The diagnostic block switched to FRAMESIZE_XGA at
 *          boot to test lens sharpness. set_framesize(XGA) reconfigured
 *          the sensor PLL successfully (confirmed by log), but the DMA
 *          descriptor ring was sized at init time for VGA and could not
 *          service an XGA frame -- esp_camera_fb_get() timed out with
 *          cam_hal "Failed to get the frame on time!" at exactly 4s (the
 *          default cam_hal frame-wait timeout).
 *
 *          The critical error was in the recovery path after that timeout:
 *          the code fell through to set_framesize(VGA) on a sensor whose
 *          DVP pipeline was already stalled mid-XGA-stream. Calling
 *          set_framesize() on a stalled pipeline does not flush or reset
 *          the parallel bus -- it only reconfigures the PLL and register
 *          file over SCCB. The DVP timing generator remained in a broken
 *          state, which is why the subsequent camera_recover() saw NO-SOI:
 *          the sensor re-detected fine on I2C but the parallel bus never
 *          produced a frame start. A full deinit+init eventually resolved
 *          it, but only after the NO-SOI verification fb_get() burned
 *          another ~4.5s and the recovery path itself approached the 5s
 *          WDT window.
 *
 *          The redundant double PLL calculation visible in the recovery
 *          log (two "Calculated VCO" lines 40ms apart) was also caused by
 *          this block: esp_camera_init() calculates PLL for VGA once, then
 *          the immediately-following set_framesize(VGA) inside camera_init()
 *          recalculates it redundantly. Harmless but now also gone since
 *          set_framesize() in camera_init() is no longer redundant with the
 *          config struct (both set FRAMESIZE_VGA, second call is a no-op
 *          in normal operation -- but emitting a second PLL log was
 *          misleading during diagnosis).
 *
 *          The diagnostic conclusion (lens sharpness at XGA vs VGA) was
 *          already obtained from the one flush frame that succeeded at
 *          1024x768 before the hang. No further diagnostic runs are needed.
 *          The block is removed entirely; VGA is the permanent operating
 *          resolution for this deployment.
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
static const char *APP_MAIN_TAG = "APP_MAIN";
/*---------------------------------------------------------------------------*/
/* Globals                                                                     */
/*---------------------------------------------------------------------------*/

static SemaphoreHandle_t g_trigger_sem = NULL;
static int               g_tcp_sock    = -1;
static bool              g_wifi_up     = false;

/* Number of stale frames to drain from the DMA ring immediately after
 * tcp_connect() returns, before starting the real clip capture loop. */
#define CAM_POST_CONNECT_FLUSH_FRAMES   2

/* How often (in ms) to pull+discard one frame while idle, to keep the
 * camera pipeline alive. Must be a multiple of the 2000ms idle wait period
 * in capture_task for the counter logic below to land on it cleanly. */
#define CAM_IDLE_PING_INTERVAL_MS        6000

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

    /* ---- Image quality tuning (2026-06-13) ----
     * Default AGC was driving high analog gain in low light, producing
     * grainy/speckled JPEGs (whole-frame noise, not blur -- ruled out
     * focus initially, since this board's OV3660 lens is fixed-focus).
     *
     * Capping gainceiling forces the sensor to stop amplifying noise:
     * the image will be clean but may go dark in low light. That's the
     * diagnostic signal -- if it's still blurry under bright light after
     * this change, it's the fixed-focus lens, not gain/exposure.
     *
     * Follow-up (2026-06-13): user confirmed room is bright and image is
     * still soft -- not a lighting/noise issue. Dropped jpeg_quality
     * 10 -> 6 (less compression, less detail loss) and added
     * set_sharpness() to counter softness via in-sensor edge enhancement.
     * set_sharpness() return value is checked since not all sensor
     * drivers implement it (returns -1/ESP_ERR_NOT_SUPPORTED if absent).
     *
     * Note: set_framesize(VGA) here is intentionally redundant with the
     * config struct above. It is kept to make camera_init() self-contained
     * when called from camera_recover() after a mode change left the sensor
     * in an unknown framesize state. The redundant PLL recalculation is
     * harmless and the log line is a useful confirmation during debugging.
     */
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
 * actually producing valid SOI/frames -- re-detecting the sensor on SCCB is
 * not sufficient evidence the parallel interface is alive ("NO-SOI" can
 * occur even after a successful re-detect, most commonly when a prior
 * failed resolution switch left the DVP pipeline stalled before deinit).
 *
 * WDT is kicked before/after every step that can block for a meaningful
 * time, since this can be called from a WDT-monitored task and each step
 * (deinit, power-cycle delays, re-init warmup, verification fb_get) can
 * individually approach or exceed the per-task WDT window.
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

    /* camera_init() includes its own 500ms warmup and re-applies all
     * sensor tuning (framesize, quality, gain ceiling, AEC, BPC/WPC). */
    esp_err_t init_err = camera_init();
    trinity_wdt_kick();

    if (ESP_OK != init_err)
    {
        ESP_LOGE(TAG, "Camera recovery failed: 0x%x", init_err);
        return init_err;
    }

    /* Verify the DVP bus is actually live before declaring success.
     * This fb_get() can block ~4.5s if the sensor re-detected on SCCB but
     * still isn't producing SOI on the parallel bus. */
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
 * camera_recover() can fail to restore a working sensor (e.g. persistent
 * NO-SOI after re-init). Retrying recovery in a tight loop just burns time
 * inside a WDT-monitored task and eventually trips the watchdog anyway (as
 * seen in the field: repeated hot-reset attempts ending in task_wdt abort).
 * If recovery doesn't produce a verified-good sensor, do a clean
 * esp_restart() instead -- this is a deterministic, bounded recovery path.
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

    /* WIFI_PS_NONE: tested, did not fix the idle-death issue on its own,
     * but left in place since it has no observed downside other than idle
     * current draw, and rules out modem-sleep as a contributing factor. */
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

        /* connect() itself can block for a while (TCP SYN timeout) if the
         * BBB is slow to accept (e.g. still draining the previous clip's
         * socket on back-to-back triggers). Kick before and after. */
        int conn_err = connect(g_tcp_sock, (struct sockaddr *)&addr, sizeof(addr));
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
        trinity_wdt_kick();
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

    /* ---- Idle keepalive (2026-06-12) ----
     * Counts elapsed idle time in units of the 2000ms xSemaphoreTake
     * timeout below, so we can ping the camera periodically while waiting
     * for a trigger. */
    uint32_t idle_ms_since_ping = 0;

    while (1)
    {
        /* ---- Trinity: bounded wait so the loop can kick the WDT while
         *      idle, waiting for a trigger.                            ---- */
        if (pdTRUE != xSemaphoreTake(g_trigger_sem, pdMS_TO_TICKS(2000)))
        {
            trinity_wdt_kick();

            /* ---- Idle keepalive (2026-06-12) ---- */
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

                /* fb_get() above (or recovery) can block; kick again
                 * immediately so that delay doesn't eat into the 5s
                 * per-task WDT window. */
                trinity_wdt_kick();
            }

            continue;
        }
        trinity_wdt_kick();

        /* Trigger received — reset the idle-ping counter so we don't
         * double-ping right after a clip finishes. */
        idle_ms_since_ping = 0;

        ESP_LOGI(TAG, "Triggered — starting clip");

        tcp_connect();
        trinity_wdt_kick();

        /* ---- Frame-capture deadlock fix (2026-06-12) ----
         * Drain any stale/garbage frames that may have queued up in the
         * DMA ring during the blocking connect() above before starting
         * the real clip. Failures here are not fatal — if the sensor is
         * already dead, the main loop below will detect it.
         */
        for (int i = 0; i < CAM_POST_CONNECT_FLUSH_FRAMES; i++)
        {
            camera_fb_t *p_flush = esp_camera_fb_get();
            if (NULL != p_flush)
            {
                esp_camera_fb_return(p_flush);
            }
        }
        trinity_wdt_kick();

        uint32_t elapsed     = 0;
        bool     sensor_dead = false;

        while (elapsed < CAM_CLIP_DURATION_MS)
        {
            if (sensor_dead)
            {
                /* Sensor already confirmed dead this clip — don't pay the
                 * ~4s fb_get() timeout on every remaining slot. Just
                 * advance the clock so the clip still ends on schedule. */
                vTaskDelay(pdMS_TO_TICKS(CAM_CLIP_FRAME_MS));
                trinity_wdt_kick();
                elapsed += CAM_CLIP_FRAME_MS;
                continue;
            }

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
                ESP_LOGE(TAG, "Capture timeout at %lu ms — marking sensor dead for this clip",
                         (unsigned long)elapsed);
                sensor_dead = true;
            }

            vTaskDelay(pdMS_TO_TICKS(CAM_CLIP_FRAME_MS));
            trinity_wdt_kick();
            elapsed += CAM_CLIP_FRAME_MS;
        }

        ESP_LOGI(TAG, "Clip complete — %lu ms", (unsigned long)elapsed);
        close(g_tcp_sock);
        g_tcp_sock = -1;

        /* ---- Frame-capture deadlock fix (2026-06-12) ----
         * If the sensor died during this clip, hot-reset it now so the
         * *next* trigger starts from a known-good state instead of
         * failing 100% of the time until power-cycle. */
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
        ESP_LOGI(APP_MAIN_TAG,
                 "[TRINITY] app_main exit stack HWM: %u words (%u B)",
                 (unsigned)hwm,
                 (unsigned)(hwm * sizeof(StackType_t)));

#ifndef CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS
#define CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS 256u
#endif
        if (hwm < (UBaseType_t)CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS)
        {
            ESP_LOGW(APP_MAIN_TAG,
                     "[TRINITY] LOW STACK on main task at exit: hwm=%u words threshold=%u words",
                     (unsigned)hwm,
                     (unsigned)CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS);
            trinity_log_record_low_stack((uint32_t)hwm);
        }
    }

}
