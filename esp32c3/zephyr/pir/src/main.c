/******************************************************************************
 * \file    main.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   ESP32-C3 PIR BLE motion sensor entry point.
 *
 * \note    WDT main-loop fix (2026-03-23):
 *          Previous code: k_sleep(K_SECONDS(10)) THEN trinity_wdt_kick().
 *          WDT timeout is 3s. The sleep fired before the kick -- device
 *          reset on every iteration in field mode. Fixed by moving the kick
 *          to the TOP of the loop before the sleep, and reducing sleep to
 *          2s so the pattern matches the other Trinity nodes. The 10s
 *          tick counter scaling is updated to preserve the original
 *          5-minute battery interval (150 ticks) and 60s stats interval
 *          (30 ticks) at the new 2s rate.
 *
 * \note    WDT boot fix (2026-03-23):
 *          trinity_wdt_kick() added after trinity_wdt_init() and after
 *          bt_enable() (~1-2s blocking call) to prevent WDT firing during
 *          boot before the main loop starts kicking.
 *
 * \note    Pre-init canary (2026-03-23):
 *          trinity_log.c now writes a .noinit canary at PRE_KERNEL_1 before
 *          any SYS_INIT driver runs. trinity_log_dump_previous() reports
 *          PRE_INIT_CRASH if the board died before trinity_log_init().
 *
 * \note    Occupied sliding window (2026-04-29):
 *          motion_count is a monotonic counter incremented by the PIR ISR.
 *          Each main-loop tick computes the delta since the previous tick
 *          and stores it in a ring buffer of WINDOW_TICKS slots.  The sum
 *          across the ring is compared to OCCUPIED_THRESH to derive a
 *          boolean occupied flag that is broadcast in MFG byte [7].
 *          One ISR in the window asserts occupied for the full window
 *          duration (WINDOW_TICKS * LOOP_SLEEP_MS ms) then clears
 *          automatically as the event rotates out the back of the ring.
 ******************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "ble_adv.h"
#include "max17048.h"
#include "trinity_log.h"
#include "main.h"

LOG_MODULE_REGISTER(pir_main, LOG_LEVEL_INF);

#define PIR_PIN    3
#define GPIO_NODE  DT_NODELABEL(gpio0)

static uint32_t motion_count = 0;
static uint8_t  batt_soc     = 0;

static struct gpio_callback pir_cb_data;

/* Sliding window state */
static uint8_t  window[WINDOW_TICKS];
static int      win_head   = 0;
static uint32_t prev_count = 0;

/*----------------------------------------------------------------------------*/

static void pir_callback(const struct device *dev,
                          struct gpio_callback *cb,
                          uint32_t pins)
{
    if (gpio_pin_get(dev, PIR_PIN))
    {
        motion_count++;
        trinity_log_event("EVENT: MOTION\n");
        LOG_INF("Motion detected! Count=%u  Batt=%d%%", motion_count, batt_soc);
    }
}

/*----------------------------------------------------------------------------*/

int main(void)
{
    int ret        = 0;
    int tick       = 0;
    int batt_tick  = 0;
    int stats_tick = 0;
    int mv         = 0;

    LOG_INF("===========================================");
    LOG_INF("  ESP32-C3 PIR BLE Motion Sensor");
    LOG_INF("===========================================");

    trinity_log_dump_previous();
    trinity_log_init();
    trinity_log_event("EVENT: BOOT\n");

    trinity_wdt_init();
    trinity_wdt_kick();

    const struct device *gpio = DEVICE_DT_GET(GPIO_NODE);
    if (!device_is_ready(gpio)) { LOG_ERR("GPIO not ready"); return 0; }
    gpio_pin_configure(gpio, PIR_PIN, GPIO_INPUT);

    /* Kick before ble_adv_init -- bt_enable() blocks ~1-2s */
    trinity_wdt_kick();
    ret = ble_adv_init();
    if (0 != ret) { return 0; }
    trinity_wdt_kick();

    max17048_init();
    batt_soc = (uint8_t)max17048_read_soc();
    mv       = max17048_read_mv();
    LOG_INF("Battery: %d%% (%d mV)", batt_soc, mv);
    ble_adv_update(motion_count, batt_soc, 0);

    gpio_init_callback(&pir_cb_data, pir_callback, BIT(PIR_PIN));
    gpio_add_callback(gpio, &pir_cb_data);
    gpio_pin_interrupt_configure(gpio, PIR_PIN, GPIO_INT_EDGE_BOTH);
    LOG_INF("PIR interrupt configured");
    LOG_INF("*** System Ready ***");

    while (1)
    {
        trinity_wdt_kick();
        k_sleep(K_MSEC(LOOP_SLEEP_MS));

        /* --- sliding window update ----------------------------------------
         * delta: events fired by ISR since the last tick.
         * Store in current ring slot, advance head, sum all slots.
         * NOTE: motion_count is written by ISR and read here without a lock.
         * On ESP32-C3 (single-core RV32) a 32-bit load is atomic.  If this
         * code is ever ported to a multi-core target use atomic_t instead.
         * ----------------------------------------------------------------- */
        uint32_t delta = motion_count - prev_count;
        prev_count = motion_count;

        window[win_head] = (uint8_t)(delta > 255U ? 255U : delta);
        win_head = (win_head + 1) % WINDOW_TICKS;

        uint32_t win_sum = 0;
        for (int i = 0; i < WINDOW_TICKS; i++) { win_sum += window[i]; }
        uint8_t occupied = (win_sum >= OCCUPIED_THRESH) ? 1U : 0U;
        /* ----------------------------------------------------------------- */

        ble_adv_update(motion_count, batt_soc, occupied);

        if (tick % 30 == 0)
        {
            LOG_INF("[%d] Motion count: %u  Batt=%d%%  Occupied=%d",
                    tick, motion_count, batt_soc, occupied);
        }
        tick++;

        if (++batt_tick >= BATT_UPDATE_TICKS)
        {
            batt_tick = 0;
            batt_soc  = (uint8_t)max17048_read_soc();
            mv        = max17048_read_mv();
            LOG_INF("Battery updated: %d%% (%d mV)", batt_soc, mv);
        }

        if (++stats_tick >= STATS_INTERVAL_TICKS)
        {
            stats_tick = 0;
            trinity_log_heap_stats();
            trinity_log_task_stats();
        }
    }

    return 0;
}
