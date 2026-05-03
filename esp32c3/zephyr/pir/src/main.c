/******************************************************************************
 * \file    main.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   ESP32-C3 PIR BLE motion sensor entry point.
 *
 * \details Deep sleep architecture. main() runs once per wake cycle,
 *          detects wake reason, advertises a burst, configures wake
 *          sources, then calls esp_deep_sleep_start(). Every wake is
 *          a full reboot -- there is no main loop.
 *
 *          Wake sources (configured every cycle):
 *          - GPIO3 (PIR)  -- event-driven, wakes on HIGH
 *          - RTC timer    -- periodic heartbeat, DEEP_SLEEP_INTERVAL_US
 *
 *          Burst duration:
 *          - PIR wake     -- ADV_BURST_PIR_MS   (2000ms)
 *          - Timer wake   -- ADV_BURST_TIMER_MS  (300ms)
 *          - First boot   -- ADV_BURST_TIMER_MS  (300ms)
 *
 *          Occupied flag:
 *          Set in MFG payload byte[7] on PIR wake, clear on timer wake.
 *          Hub owns all occupancy window and timeout logic -- no sliding
 *          window on device. State resets every wake by design.
 *
 * \note    Architecture (2026-05-03):
 *          Sliding window occupancy logic moved to hub.
 *          Motivation: always-on BLE stack idles at ~52mA, limiting a
 *          3000mAh cell to ~57 hours (2.4 days). Deep sleep architecture
 *          measured at 19.93mA average over full wake/advertise/sleep
 *          cycles (PPK2, nRF Power Profiler), extending runtime to
 *          ~150 hours (~6.3 days) on the same cell. 2.6x improvement.
 *
 *          Device is now stateless per wake:
 *          - PIR wake   -> occupied=1, motion_count++, 2s burst, sleep
 *          - Timer wake -> occupied=0, 300ms burst, sleep
 *          Hub uses motion_count (monotonic, NVS-persistent) to detect
 *          missed BLE packets and owns all occupancy timeout logic.
 *          No window state is kept on device -- intentional by design.
 *
 * \note    WDT (2026-03-23):
 *          trinity_wdt_kick() called before any blocking operation.
 *          WDT timeout is 3s. Kick is issued immediately before the
 *          burst sleep to isolate the window to burst_ms only.
 *          Mid-burst kick fires if burst_ms >= WDT_TIMEOUT_MS.
 *
 * \note    Deep sleep power (2026-05-03):
 *          Always-on BLE advertising on ESP32-C3 idles at ~52mA due to
 *          the BLE stack requiring CPU involvement. Deep sleep drops
 *          average current to 19.93mA measured over real wake cycles.
 *          esp_deep_sleep_start() is __noreturn__ -- every wake is a
 *          full cold boot. Zephyr PM layer (CONFIG_PM) does not reach
 *          true deep sleep on ESP32-C3; use ESP-IDF API directly.
 *          CONFIG_PM=n in bench.conf -- USB console is incompatible with
 *          deep sleep (USB peripheral loses power during sleep).
 *
 * \note    Motion count persistence (2026-05-03):
 *          motion_count persisted via NVS (trinity_nvs_esp.c) on every
 *          confirmed PIR wake. Survives deep sleep cycles, resets only
 *          on power-on or hard reset.
 *
 *          RTC_DATA_ATTR was attempted first and does not work on this
 *          toolchain/Zephyr version. See CHANGELOG below for full details.
 *
 *          Hub uses delta between received motion_count values to detect
 *          missed BLE packets during lossy periods.
 *          All other state resets on every wake by design.
 *
 *****************************************************************************
 * CHANGELOG
 *
 * 2026-05-03 — Hardware: AM312 PIR false trigger fix
 *   PROBLEM:  GPIO3 read 1.28V only when physically touching the wire or
 *             board -- not the PIR firing. Body capacitance was sufficient
 *             to lift a floating pin to ~1.28V, which crossed the ESP32-C3
 *             GPIO HIGH threshold and triggered a spurious wake. A properly
 *             driven signal would jump to 2.6-3V and hold there -- the
 *             1.28V that only appeared on touch confirmed no actual driver
 *             on the line.
 *             AM312 OUT measured 2.6V when disconnected from ESP32,
 *             collapsed to 1.28V the moment it was connected to GPIO3.
 *             Root cause: AM312 output is essentially open-drain with a
 *             weak internal pull-up. It cannot source enough current to
 *             overcome the ESP32-C3 internal pull-down (~45kΩ). The two
 *             were fighting and the pull-down was winning -- any real
 *             motion signal was also being suppressed.
 *   FIX:      Disabled the internal pull-down on GPIO3. AM312 now drives
 *             cleanly to 2.6-3V on motion, returns to 0V at idle.
 *             Debounce check (10ms settle + PIN re-read) retained in
 *             software to catch any residual transient wakes.
 *
 * 2026-05-03 — Architecture: always-on BLE -> deep sleep
 *   PROBLEM:  Always-on BLE advertising required continuous CPU involvement
 *             on ESP32-C3, idling at ~52mA average. On a 3000mAh cell this
 *             gives ~57 hours (2.4 days) of runtime.
 *   FIX:      Replaced always-on loop with deep sleep architecture.
 *             main() runs once per wake cycle, advertises a short burst,
 *             then calls esp_deep_sleep_start() (noreturn). Every wake is
 *             a full cold boot. Wake sources: GPIO3 (PIR) and RTC timer
 *             (10s heartbeat). Average current measured at 19.93mA over
 *             a 3m49s sample window capturing full wake/advertise/sleep
 *             cycles (PPK2, nRF Power Profiler). Runtime on 3000mAh cell:
 *             ~150 hours (~6.3 days). 2.6x improvement over always-on.
 *             CONFIG_PM=n -- Zephyr PM layer does not reach true deep sleep
 *             on ESP32-C3. ESP-IDF esp_deep_sleep_start() used directly.
 *
 * 2026-05-03 — Bug: motion_count not persisting across deep sleep
 *   PROBLEM:  motion_count declared as `static RTC_DATA_ATTR uint32_t`.
 *             The `static` storage class caused the compiler to commit the
 *             symbol to DRAM first; RTC_DATA_ATTR section attribute was
 *             silently ignored. Boot log confirmed: `load:0x50000000,
 *             len:0xc` (12 bytes in RTC -- only the wake stub, not
 *             motion_count). DRAM is reloaded from flash on every deep
 *             sleep wake so motion_count reset to 0 every boot and count
 *             always showed 1.
 *             Root cause: Zephyr ESP32-C3 linker script has no proper
 *             .rtc.data section. RTC_DATA_ATTR is a no-op for data
 *             variables in this toolchain/Zephyr version combination.
 *             Removing `static` was also attempted and did not change
 *             placement -- confirmed by boot log showing len:0xc unchanged.
 *   FIX:      Removed RTC_DATA_ATTR entirely. Added trinity_nvs_esp.c --
 *             a dedicated NVS persistence module using storage_partition
 *             (0x3b0000, 176KB). NVS is mounted independently of trinity
 *             flash log (which uses log_partition -- no overlap, no
 *             collision risk). motion_count is read from NVS on every
 *             wake and written to NVS only on confirmed PIR wakes.
 *             Flash wear is negligible at even 100 events/day.
 *             count now accumulates correctly across all deep sleep cycles.
 * 2026-05-03 — Hardware: AM312 false triggers on battery power
 *   PROBLEM:  PIR count incrementing with no movement when running on
 *             battery. AM312 was physically close to the LiPo cell.
 *             Battery warms under load (BLE TX, flash writes) and the
 *             thermal change falls within the AM312 detection cone.
 *             PIR fired on the battery's IR signature -- a real detection
 *             of a real heat source, not an electrical false trigger.
 *             Did not occur on USB power because battery stays at ambient
 *             temperature when not under load.
 *   FIX:      Moved AM312 away from battery. Keep PIR and LiPo thermally
 *             separated in final enclosure. If space is constrained, a
 *             small aluminum tape barrier between battery and PIR face
 *             will block the IR path.
 ******************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <esp_sleep.h>
#include "ble_adv.h"
#include "max17048.h"
#include "trinity_log.h"
#include "main.h"

LOG_MODULE_REGISTER(pir_main, LOG_LEVEL_INF);

#define PIR_PIN   3
#define GPIO_NODE DT_NODELABEL(gpio0)

/*----------------------------------------------------------------------------*/

static void enter_deep_sleep(void)
{
    /* Both wake sources re-armed every cycle -- deep sleep clears all
     * wake config on exit. GPIO3 for motion events, RTC timer for
     * periodic heartbeat so hub can detect device loss. */
    esp_deep_sleep_enable_gpio_wakeup(1ULL << PIR_PIN,
                                       ESP_GPIO_WAKEUP_GPIO_HIGH);
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_INTERVAL_US);

    LOG_INF("Entering deep sleep (timer=%llu us, GPIO%d wake)",
            (unsigned long long)DEEP_SLEEP_INTERVAL_US, PIR_PIN);

    /* Drain deferred log synchronously before sleep.
     * CONFIG_LOG_PROCESS_THREAD_SLEEP_MS=1000 means the log thread will
     * not flush before esp_deep_sleep_start() destroys RAM. */
    trinity_log_flush();

    esp_deep_sleep_start(); /* noreturn -- next line never executes */
}

/*----------------------------------------------------------------------------*/

int main(void)
{
    int      ret       = 0;
    int      mv        = 0;
    uint8_t  batt_soc  = 0;
    uint8_t  occupied  = 0;
    uint32_t burst_ms  = 0;

    esp_sleep_wakeup_cause_t wake_reason = esp_sleep_get_wakeup_cause();

    LOG_INF("===========================================");
    LOG_INF("  ESP32-C3 PIR BLE Motion Sensor");
    LOG_INF("  Wake: %d", (int)wake_reason);
    LOG_INF("===========================================");

    trinity_log_dump_previous();
    trinity_log_init();
    trinity_wdt_init();
    trinity_wdt_kick();

    trinity_nvs_init();
    uint32_t motion_count = trinity_nvs_read_motion_count();

    const struct device *gpio = DEVICE_DT_GET(GPIO_NODE);
    if (!device_is_ready(gpio))
    {
        LOG_ERR("GPIO not ready");
        enter_deep_sleep();
        return 0;
    }
    gpio_pin_configure(gpio, PIR_PIN, GPIO_INPUT);

    /* Determine burst duration and occupied flag from wake reason */
    switch (wake_reason)
    {
        case ESP_SLEEP_WAKEUP_GPIO:
            /* Debounce: confirm PIN is still HIGH after settle time.
             * Rules out transient pulls during GPIO peripheral init
             * that can misfire as a GPIO wake on some timer wakes.
             *
             * Also catches body capacitance false triggers -- a floating
             * or weakly driven pin can be lifted by proximity/touch to
             * ~1.28V which crosses the GPIO HIGH threshold. A real PIR
             * signal holds 2.6-3V; anything that drops on re-read is
             * rejected here. See CHANGELOG: AM312 false trigger fix. */
            k_sleep(K_MSEC(10));
            if (gpio_pin_get(gpio, PIR_PIN) != 1)
            {
                /* False wake -- treat as timer heartbeat */
                LOG_WRN("GPIO wake but PIN low -- false trigger, treating as heartbeat");
                occupied = 0;
                burst_ms = ADV_BURST_TIMER_MS;
                break;
            }
            motion_count++;
            trinity_nvs_write_motion_count(motion_count);
            occupied = 1;   /* hub starts/extends occupancy window on receipt */
            burst_ms = ADV_BURST_PIR_MS;
            trinity_log_event("EVENT: MOTION\n");
            LOG_INF("PIR wake -- motion detected (count=%u)", motion_count);
            break;

        case ESP_SLEEP_WAKEUP_TIMER:
            occupied = 0;   /* hub expires occupancy window after DEEP_SLEEP_INTERVAL_US + margin */
            burst_ms = ADV_BURST_TIMER_MS;
            LOG_INF("Timer wake -- heartbeat");
            break;

        default:
            /* First boot or undefined wake */
            occupied = 0;
            burst_ms = ADV_BURST_TIMER_MS;
            trinity_log_event("EVENT: BOOT\n");
            LOG_INF("Cold boot");
            break;
    }

    /* BLE init and start advertising */
    trinity_wdt_kick();
    ret = ble_adv_init();
    if (0 != ret)
    {
        LOG_ERR("BLE init failed (%d) -- sleeping", ret);
        enter_deep_sleep();
        return 0;
    }
    trinity_wdt_kick();

    /* Battery read */
    max17048_init();
    batt_soc = (uint8_t)max17048_read_soc();
    mv       = max17048_read_mv();
    LOG_INF("Battery: %d%% (%d mV)", batt_soc, mv);

    /* Update advertisement payload -- must be called before any hub
     * can act on the data. ble_adv_init() starts advertising with
     * zeroed payload; this overwrites it with correct values. */
    ble_adv_update(motion_count, batt_soc, occupied);
    LOG_INF("Advertising %ums | count=%u occ=%d batt=%d%%",
            burst_ms, motion_count, occupied, batt_soc);

    /* Kick immediately before burst sleep to give the full WDT_TIMEOUT_MS
     * budget to the burst. Mid-burst kick required if burst >= WDT timeout. */
    trinity_wdt_kick();
    if (burst_ms >= WDT_TIMEOUT_MS)   /* fix: was > 2000U, missed exact 2000ms PIR burst */
    {
        k_sleep(K_MSEC(burst_ms / 2U));
        trinity_wdt_kick();
        k_sleep(K_MSEC(burst_ms / 2U));
    }
    else
    {
        k_sleep(K_MSEC(burst_ms));
    }

    trinity_wdt_kick();

    /* Shut down BLE before entering deep sleep */
    bt_disable();

    enter_deep_sleep(); /* noreturn */

    return 0; /* unreachable */
}
