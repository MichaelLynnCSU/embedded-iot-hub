/******************************************************************************
 * \file    main.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    2026-06-02
 *
 * \brief   Temperature sensor node entry point for ambient monitoring.
 *
 * \details Polls TMP36 via SAADC channel 1 (AIN1, P0.03) on a fixed
 *          interval and broadcasts temperature in tenths of °C over BLE
 *          manufacturer data advertisements. Battery SOC is included in
 *          the advertisement payload and updated every 5 minutes.
 *
 *          Hardware:
 *          - Board:    Teyleten Robot Pro Micro nRF52840
 *          - TMP36:    OUT → P0.02 (AIN1), VCC → 2.5V LDO, GND → GND
 *          - Battery:  MAX17048 fuel gauge over I2C1 (P1.13 SDA, P1.15 SCL)
 *
 * \note    No GPIO interrupt needed -- TMP36 is polled on TEMP_POLL_MS
 *          interval via k_sleep(). No semaphore, no debounce, no callback.
 *
 * \note    WDT:
 *          trinity_wdt_kick() called at top of temp_monitor_loop() and at
 *          each boot checkpoint. TEMP_POLL_MS (2000ms) is well inside the
 *          WDT window.
 *
 * \note    RESETREAS clear fix (inherited from reed-sensor 2026-03-27):
 *          RESETREAS read and cleared at very top of main() before any
 *          other code runs. Register is latched OR-history -- must be
 *          explicitly cleared to prevent stale bits surviving across boots.
 *
 * \note    Battery read before BLE (inherited from reed-sensor 2026-04-22):
 *          battery_read_soc() called before bt_enable() while VDD is
 *          stable. bt_enable() current spike can affect SAADC rail.
 *          Pre-populated into g_mfg_data via ble_adv_set_batt() so the
 *          first advertisement always carries a valid SOC.
 *
 * \note    Initial temp read before BLE thread:
 *          temp_read_decidegc() called before bt_enable() and pushed into
 *          g_ble_msgq so the first advertisement carries a real reading.
 ******************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <string.h>
#include "ble_adv.h"
#include "battery.h"
#include "temp.h"
#include "trinity_log.h"
#include "main.h"

LOG_MODULE_REGISTER(temp_main, LOG_LEVEL_INF);

#define BLE_STACK_SIZE  4096
#define BLE_PRIORITY    5

/*----------------------------------------------------------------------------*/

static void temp_monitor_loop(void)
{
    int     err        = 0;
    int     batt_tick  = 0;
    int     stats_tick = 0;
    int16_t temp       = 0;
    uint8_t soc        = 0;

    while (1)
    {
        trinity_wdt_kick();

        k_sleep(K_MSEC(TEMP_POLL_MS));

        if (++batt_tick >= BATT_UPDATE_TICKS)
        {
            batt_tick = 0;
            soc = battery_read_soc();
            ble_adv_set_batt(soc);
            LOG_INF("[BATT] SOC=%d%%", soc);
        }

        if (++stats_tick >= STATS_INTERVAL_TICKS)
        {
            stats_tick = 0;
            trinity_log_heap_stats();
            trinity_log_task_stats();
        }

        temp = temp_read_decidegc();
        if (TEMP_READ_ERROR == temp)
        {
            LOG_WRN("[TEMP] Read error, skipping broadcast");
            continue;
        }

        err = k_msgq_put(&g_ble_msgq, &temp, K_NO_WAIT);
        if (0 != err) { LOG_WRN("[TEMP] msgq full, drop (err=%d)", err); }
    }
}

/*----------------------------------------------------------------------------*/

int main(void)
{
    int      err          = 0;
    int16_t  init_temp    = 0;
    uint8_t  soc          = 0;
    uint32_t reset_reason = 0;
    k_tid_t  tid;

    static K_THREAD_STACK_DEFINE(ble_stack, BLE_STACK_SIZE);
    static struct k_thread ble_thread_data;

    reset_reason = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = 0xFFFFFFFF;

#if !defined(CONFIG_TRINITY_MODE_BENCH)
    g_init_stage = TRINITY_STAGE_WDT_INIT;
    trinity_wdt_init();
    trinity_wdt_kick();
#endif

    LOG_INF("=== TempSensor Boot ===");
    trinity_log_dump_previous();

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

    if (0 != battery_init()) { LOG_WRN("Battery init failed, SOC will read 0"); }
    trinity_wdt_kick();

    if (0 != temp_init()) { LOG_WRN("Temp init failed, readings will error"); }
    trinity_wdt_kick();

    /* Read battery before BLE -- VDD stable, radio idle. */
    soc = battery_read_soc();
    ble_adv_set_batt(soc);
    LOG_INF("Pre-BLE battery mV: %d", battery_read_mv());
    LOG_INF("Pre-BLE battery SOC: %d%%", soc);
    trinity_wdt_kick();

    /* Read initial temperature before BLE -- SAADC stable, radio idle. */
    init_temp = temp_read_decidegc();
    if (TEMP_READ_ERROR != init_temp)
    {
        LOG_INF("Pre-BLE temp: %d.%d°C",
                (int)(init_temp / 10),
                (int)(init_temp < 0 ? -(init_temp % 10) : init_temp % 10));

        err = k_msgq_put(&g_ble_msgq, &init_temp, K_NO_WAIT);
        if (0 != err) { LOG_WRN("Initial temp msgq put failed (err=%d)", err); }
    }
    else
    {
        LOG_WRN("Initial temp read failed");
    }
    trinity_wdt_kick();

    tid = k_thread_create(&ble_thread_data,
                           ble_stack, BLE_STACK_SIZE,
                           ble_thread,
                           NULL, NULL, NULL,
                           BLE_PRIORITY, 0, K_NO_WAIT);
    if (NULL == tid) { LOG_ERR("BLE thread create failed"); return -ENOMEM; }
    k_thread_name_set(tid, "ble_thread");

    trinity_wdt_kick();

    LOG_INF("BLE thread spawned");
    LOG_INF("Monitoring temperature");

    temp_monitor_loop();

    return 0;
}
