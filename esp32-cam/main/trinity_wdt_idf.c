/******************************************************************************
 * \file trinity_wdt_idf.c
 * \brief Trinity task watchdog -- ESP-IDF (motor + hub).
 *
 * \details Fully standalone. Uses esp_task_wdt with trigger_panic=true.
 *          CONFIG_ESP_TASK_WDT_PANIC=y required in sdkconfig.defaults --
 *          without it the WDT fires a warning but does NOT restart or call
 *          the panic handler.
 *
 *          Bench mode: WDT skipped entirely. USB-JTAG halts the CPU which
 *          would fire the task WDT. OpenOCD is the safety net.
 *
 *          Per-task registration: each task must call trinity_wdt_add() then
 *          trinity_wdt_kick() regularly. Unlike nRF52840 there is no single
 *          channel 0 -- each task registers independently.
 *
 *          Stack high-water-mark monitoring: every trinity_wdt_kick() call
 *          samples the calling task's remaining stack. If the margin falls
 *          below TRINITY_STACK_LOW_WATERMARK_WORDS, an ESP_LOGW is emitted
 *          and the event is persisted to the Trinity NVS fault log via
 *          trinity_log_record_low_stack() so trinity_log_dump_previous()
 *          surfaces it on the next boot, even if the device subsequently
 *          crashes before the overflow manifests as a heap assert.
 *
 *          NVS write rate: the low-stack NVS record is written AT MOST ONCE
 *          per boot per task via a task-local static flag. ESP_LOGW still
 *          fires on every kick while the condition persists (so the serial
 *          log shows the trend), but flash is not burned repeatedly.
 ******************************************************************************/
#include "trinity_log.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdbool.h>

#define WDT_TIMEOUT_S  5u

/**
 * Minimum acceptable stack headroom in words (1 word = 4 bytes on both
 * Xtensa/ESP32/ESP32-S3 and RISC-V/ESP32-C3).
 *
 * uxTaskGetStackHighWaterMark() always returns WORDS, never bytes.
 * All comparisons, log messages, and NVS records in this file use words.
 * Multiply by sizeof(StackType_t) == 4 to convert to bytes if needed.
 *
 * Tune via sdkconfig: CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS (default 256).
 *   >512 words  comfortable
 *   256–512     watch
 *   <256        warning (threshold)
 *   <128        immediate action
 */
#ifndef CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS
#define CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS  256u
#endif
#define STACK_LOW_WORDS  ((UBaseType_t)CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS)

static const char *TAG = "TRINITY_WDT";

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/**
 * \brief Sample the calling task's stack HWM and warn+persist if low.
 *
 * Called from trinity_wdt_kick() so every Trinity-monitored task gets
 * checked on every watchdog period without any additional call-sites.
 *
 * NVS write policy: a task-local static flag gates the NVS write to once
 * per boot. ESP_LOGW fires on every kick while the condition holds so the
 * serial log shows whether margin is recovering or still degraded, without
 * burning flash on every kick cycle.
 *
 * Task name: captured here via pcTaskGetName(NULL) rather than passed in
 * from the call-site. This keeps the API surface of
 * trinity_log_record_low_stack() narrow (hwm only) and removes any risk of
 * a stale or caller-supplied pointer reaching the logger.
 */
static void check_stack_hwm(void)
{
    /* Once-per-boot NVS write gate. Task-local because each task that calls
     * trinity_wdt_kick() gets its own copy of this function frame on its own
     * stack -- but that's not sufficient: multiple calls on the SAME task
     * need the flag to persist across invocations.  A static local does that
     * correctly for a single-task caller; if multiple tasks share one kick
     * path (unusual but possible) they'll each see their own static, which
     * is also correct. */
    static bool s_nvs_written = false;

    UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);  /* NULL = calling task */

    if (hwm < STACK_LOW_WORDS)
    {
        /* Always log to serial so the operator can see the trend across kicks. */
        ESP_LOGW(TAG,
                 "[TRINITY] LOW STACK: task='%s' hwm=%u words (%u bytes) threshold=%u words",
                 pcTaskGetName(NULL),
                 (unsigned)hwm,
                 (unsigned)(hwm * sizeof(StackType_t)),   /* sizeof(StackType_t)==4 always */
                 (unsigned)STACK_LOW_WORDS);

        /* Serial log (above) = runtime telemetry: ungated so the trend is
         * visible across every kick while the condition persists.
         * NVS fault record (below) = persistent forensic evidence: written
         * once per boot.  Different lifecycles, different rate limits. */
        if (!s_nvs_written)
        {
            s_nvs_written = true;
            /* Task name and units captured here; trinity_log_record_low_stack()
             * receives only the hwm value and resolves the name itself. */
            trinity_log_record_low_stack((uint32_t)hwm);
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void trinity_wdt_init(void)
{
#if defined(CONFIG_TRINITY_MODE_BENCH) && CONFIG_TRINITY_MODE_BENCH
    ESP_LOGI(TAG, "[TRINITY] WDT skipped (bench mode)");
    return;
#endif
    esp_task_wdt_config_t cfg = {
        .timeout_ms     = WDT_TIMEOUT_S * 1000u,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_err_t err = esp_task_wdt_reconfigure(&cfg);
    if (ESP_OK != err)
    {
        ESP_LOGW(TAG, "Task WDT reconfigure failed: %s", esp_err_to_name(err));
        trinity_log_event("EVENT: WDT_INIT_FAIL\n");
        return;
    }
    ESP_LOGI(TAG, "[TRINITY] Task WDT armed (%u s, panic=true)", WDT_TIMEOUT_S);
    trinity_log_event("EVENT: WDT_ARMED\n");
}

void trinity_wdt_add(void)
{
#if defined(CONFIG_TRINITY_MODE_BENCH) && CONFIG_TRINITY_MODE_BENCH
    return;
#endif
    esp_err_t err = esp_task_wdt_add(NULL);
    if (ESP_OK != err)
    {
        ESP_LOGW(TAG, "Task WDT add failed: %s", esp_err_to_name(err));
    }
}

void trinity_wdt_kick(void)
{
#if defined(CONFIG_TRINITY_MODE_BENCH) && CONFIG_TRINITY_MODE_BENCH
    return;
#endif
    check_stack_hwm();   /* sample HWM; NVS write gated to once-per-boot */
    (void)esp_task_wdt_reset();
}
