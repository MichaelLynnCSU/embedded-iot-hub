/******************************************************************************
 * \file    main.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   nRF52840 smart lock entry point.
 *
 * \note    WDT main-loop fix (2026-03-23):
 *          WDT kicked unconditionally every 2s in main loop.
 *
 * \note    Field boot WDT fix (2026-03-31):
 *          Field/USB: trinity_wdt_init() before first printk -- UART TX
 *          blocks long enough for bootloader WDT CRV=1 to fire.
 *          Bench: trinity_wdt_init() after trinity_log_init().
 *
 * \note    Deferred dump fix (2026-03-31):
 *          trinity_log_dump_previous() -- bench only, RTT-safe, called early.
 *          trinity_log_dump_previous_deferred() -- field/USB only, UART-safe,
 *          called after WDT is being kicked regularly.
 *
 * \note    NVS WDT kicks (2026-03-31):
 *          trinity_wdt_kick() before and after settings_subsys_init().
 *          Flash erase on first boot exceeds bootloader WDT window.
 *
 * \note    RESETREAS clear fix (2026-03-27):
 *          Read and cleared at very top of main() before anything else.
 *          Latched OR-history register -- stale bits survive across boots
 *          without explicit clear.
 *
 * \note    Flash concurrency fix (2026-03-24):
 *          save_lock_state() in ble_gatt.c wraps settings_save_one()
 *          with trinity_flash_lock/unlock(). See ble_gatt.c for details.
 ******************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <nrf.h>
#include "ble_gatt.h"
#include "motor.h"
#include "battery.h"
#include "trinity_log.h"
#include "config.h"

LOG_MODULE_REGISTER(smartlock_main, LOG_LEVEL_DBG);

#define LIGHT_NODE  DT_ALIAS(led0)

static const struct gpio_dt_spec g_led = GPIO_DT_SPEC_GET(LIGHT_NODE, gpios);
static bool     g_nvs_available = false;
static uint8_t  g_lock_state    = 0;

/*----------------------------------------------------------------------------*/

static int lock_state_set(const char *key, size_t len,
                           settings_read_cb read_cb, void *cb_arg)
{
    if (len != sizeof(g_lock_state)) { return -EINVAL; }
    read_cb(cb_arg, &g_lock_state, sizeof(g_lock_state));
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(lock, "lock", NULL, lock_state_set, NULL, NULL);

/*----------------------------------------------------------------------------*/

int main(void)
{
    int      err          = 0;
    int      mv           = 0;
    uint8_t  soc          = 0;
    uint32_t reset_reason = 0;

    reset_reason         = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = 0xFFFFFFFF;

#if !defined(CONFIG_TRINITY_MODE_BENCH)
    g_init_stage = TRINITY_STAGE_WDT_INIT;
    trinity_wdt_init();
    trinity_wdt_kick();
#endif

    printk("=== SmartLock Boot ===\n");

    trinity_log_dump_previous();

    g_init_stage = TRINITY_STAGE_GPIO;

    if (!device_is_ready(g_led.port)) { LOG_ERR("LED not ready"); return -ENODEV; }
    gpio_pin_configure_dt(&g_led, GPIO_OUTPUT_INACTIVE);

    err = motor_init();
    if (0 != err) { return err; }

    g_init_stage = TRINITY_STAGE_BATTERY;

    if (0 == battery_init())
    {
        mv = battery_read_mv();
        if (0 < mv)
        {
            soc = mv_to_soc(mv);
            printk("[BATT] Initial: %d%% (%d mV)\n", soc, mv);
        }
    }
    else
    {
        printk("[BATT] Init failed\n");
    }

    g_init_stage = TRINITY_STAGE_NVS_INIT;

    trinity_wdt_kick();
    if (0 == settings_subsys_init())
    {
        g_nvs_available = true;
        trinity_wdt_kick();

        g_init_stage = TRINITY_STAGE_NVS_LOAD;

        settings_load();
        printk("NVS ready, lock_state=%d\n", g_lock_state);
    }
    else
    {
        LOG_WRN("NVS init failed");
    }
    trinity_wdt_kick();

    gpio_pin_set_dt(&g_led, g_lock_state ? 1 : 0);

    g_init_stage = TRINITY_STAGE_LOG_INIT;

    if (0 == trinity_log_init())
    {
        trinity_log_boot_reason(reset_reason);
        battery_print_status();
    }
    trinity_wdt_kick();

    trinity_log_dump_previous_deferred();

#if defined(CONFIG_TRINITY_MODE_BENCH)
    g_init_stage = TRINITY_STAGE_WDT_INIT;
    trinity_wdt_init();
    trinity_wdt_kick();
#endif

    g_init_stage = TRINITY_STAGE_BT_ENABLE;

    /* Push NVS-loaded state and initial battery into BLE layer before
     * ble_gatt_init() starts advertising so first advertisement is correct. */
    ble_set_lock_state(g_lock_state);
    ble_set_batt(soc);

    err = ble_gatt_init();
    if (0 != err) { return err; }

    g_init_stage = TRINITY_STAGE_MAIN_LOOP;

    while (1)
    {
        trinity_wdt_kick();
        k_sleep(K_MSEC(2000));
    }

    return 0;
}
