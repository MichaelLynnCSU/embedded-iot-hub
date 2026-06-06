/******************************************************************************
 * \file    main.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   nRF52840 smart light relay controller entry point.
 *
 * \note    settings_load() fix (2026-03-24):
 *          Called after bt_enable() in ble_gatt_init() so BLE stack has
 *          a valid identity address and bt_le_adv_start() does not hang.
 *
 * \note    Init stage cookie (2026-03-24):
 *          g_init_stage breadcrumbs written before each risky init call.
 *          Printed alongside PRE_INIT_CRASH to narrow crash to one call.
 *
 * \note    WDT boot fix (2026-03-24):
 *          trinity_wdt_kick() added after bt_enable() and settings_load()
 *          inside ble_gatt_init() to prevent WDT firing during boot.
 *
 * \note    Flash concurrency fix (2026-03-24):
 *          write_light_control() calls trinity_log_event() which acquires
 *          the flash mutex internally -- no external lock needed.
 *
 * \note    RESETREAS clear fix (2026-03-27):
 *          RESETREAS read and cleared at very top of main() before any
 *          other code runs. Latched OR-history register -- must be
 *          explicitly cleared to prevent stale bits surviving across boots.
 *
 * \note    GPIO seam (2026-05-xx):
 *          All GPIO access moved to light_hw_zephyr.c behind light_hw.h.
 *          main.c has no GPIO or devicetree dependency.
 *          relay_set() is a thin wrapper over light_hw_set() so ble_gatt.c
 *          calling relay_set() requires no changes.
 ******************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <nrf.h>
#include "light_hw.h"
#include "ble_gatt.h"
#include "trinity_log.h"
#include "main.h"

int main(void)
{
    int      err          = 0;
    uint32_t reset_reason = 0;

    reset_reason         = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = 0xFFFFFFFF;

#if !defined(CONFIG_TRINITY_MODE_BENCH)
    g_init_stage = TRINITY_STAGE_WDT_INIT;
    trinity_wdt_init();
    trinity_wdt_kick();
#endif

    printk("\n=== LightNF BLE Relay Controller ===\n");

    trinity_log_dump_previous();

    g_init_stage = TRINITY_STAGE_GPIO;
    err = light_hw_init();
    if (0 != err) { return err; }

    g_init_stage = TRINITY_STAGE_LOG_INIT;
    if (trinity_log_init() == 0)
    {
        trinity_log_boot_reason(reset_reason);
    }
    trinity_wdt_kick();

    trinity_log_dump_previous_deferred();

#if defined(CONFIG_TRINITY_MODE_BENCH)
    g_init_stage = TRINITY_STAGE_WDT_INIT;
    trinity_wdt_init();
    trinity_wdt_kick();
#endif

    g_init_stage = TRINITY_STAGE_BT_ENABLE;
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
