/******************************************************************************
 * \file    main.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Reed sensor node entry point for door/window state monitoring.
 *
 * \details Monitors a reed switch via GPIO interrupt and broadcasts
 *          open/closed state over BLE manufacturer data advertisements.
 *          Battery SOC is included in the advertisement payload and
 *          updated every 5 minutes.
 *
 *          Hardware:
 *          - Board:    Pro Micro nRF52840
 *          - Reed pin: GPIO0 pin 11 (active-low, pull-up)
 *          - LED:      DT alias led0
 *          - Battery:  MAX17048 fuel gauge over I2C1 (P1.13 SDA, P1.15 SCL)
 *
 * \note    WDT fix (2026-03-21):
 *          REED_POLL_MS reduced from 10000ms to 2000ms. trinity_wdt_kick()
 *          called before sem_take so WDT is fed even when reed is stable.
 *
 * \note    WDT boot-feed fix (2026-03-22):
 *          Progressive WDT feeding at each boot checkpoint. bt_enable()
 *          and settings_load() can each consume up to 2s of the 3s budget.
 *
 * \note    RESETREAS clear fix (2026-03-27):
 *          RESETREAS read and cleared at very top of main() before any
 *          other code runs. Register is latched OR-history -- must be
 *          explicitly cleared to prevent stale bits surviving across boots.
 *
 * \note    Battery read before BLE (2026-04-22, updated 2026-05-01):
 *          battery_read_soc() called before bt_enable() while VDD is
 *          stable. bt_enable() current spike collapses CR2032 rail and
 *          corrupts SAADC reading. Pre-populated into g_mfg_data via
 *          ble_adv_set_batt() so BLE thread uses it directly. With
 *          MAX17048 the rail collapse is less critical (fuel gauge is
 *          independent of SAADC) but the ordering is retained to ensure
 *          the first advertisement always carries a valid SOC.
 *
 * \note    Battery divider removed (2026-04-22):
 *          Resistor divider physically removed. Vbat connects directly
 *          to P0.02 (AIN0). CR2032 3.0V is within ADC_GAIN_1_6 max
 *          input of 3.6V. No quiescent divider drain.
 ******************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <string.h>
#include "ble_adv.h"
#include "battery.h"
#include "trinity_log.h"
#include "main.h"

LOG_MODULE_REGISTER(reed_main, LOG_LEVEL_INF);

#define LED_NODE    DT_ALIAS(led0)
#define REED_PORT   DT_NODELABEL(gpio0)
#define REED_PIN    11

#define BLE_STACK_SIZE   4096
#define BLE_PRIORITY     5
#define REED_DEBOUNCE_MS 50

static const struct gpio_dt_spec g_led =
    GPIO_DT_SPEC_GET(LED_NODE, gpios);
static const struct device *g_gpio0;

K_SEM_DEFINE(g_reed_changed_sem, 0, 1);

static struct gpio_callback g_reed_cb_data;

/*----------------------------------------------------------------------------*/

static void reed_interrupt_handler(const struct device *p_dev,
                                    struct gpio_callback *p_cb,
                                    uint32_t pins)
{
    (void)p_dev;
    (void)p_cb;
    (void)pins;

    k_sem_give(&g_reed_changed_sem);
}

/*----------------------------------------------------------------------------*/

static int gpio_init(void)
{
    int err = 0;

    g_gpio0 = DEVICE_DT_GET(REED_PORT);

    if (!device_is_ready(g_gpio0))    { LOG_ERR("GPIO0 not ready");  return -ENODEV; }
    if (!device_is_ready(g_led.port)) { LOG_ERR("LED not ready");    return -ENODEV; }

    err = gpio_pin_configure_dt(&g_led, GPIO_OUTPUT_INACTIVE);
    if (0 != err) { LOG_ERR("LED configure failed (err=%d)", err); return err; }

    err = gpio_pin_configure(g_gpio0, REED_PIN, GPIO_INPUT | GPIO_PULL_UP);
    if (0 != err) { LOG_ERR("Reed pin configure failed (err=%d)", err); return err; }

    err = gpio_pin_interrupt_configure(g_gpio0, REED_PIN, GPIO_INT_EDGE_BOTH);
    if (0 != err) { LOG_ERR("Reed interrupt configure failed (err=%d)", err); return err; }

    gpio_init_callback(&g_reed_cb_data, reed_interrupt_handler, BIT(REED_PIN));

    err = gpio_add_callback(g_gpio0, &g_reed_cb_data);
    if (0 != err) { LOG_ERR("Reed callback add failed (err=%d)", err); return err; }

    return 0;
}

/*----------------------------------------------------------------------------*/

static void reed_monitor_loop(int last_state)
{
    int     err        = 0;
    int     val        = 0;
    int     batt_tick  = 0;
    int     stats_tick = 0;
    uint8_t cur        = 0;
    uint8_t state      = 0;
    uint8_t soc        = 0;

    while (1)
    {
        trinity_wdt_kick();

        (void)k_sem_take(&g_reed_changed_sem,
                          K_TIMEOUT_ABS_TICKS(
                             k_uptime_ticks() +
                             k_ms_to_ticks_ceil32(REED_POLL_MS)));

        if (++batt_tick >= BATT_UPDATE_TICKS)
        {
            batt_tick = 0;
            soc = battery_read_soc();
            ble_adv_set_batt(soc);
            cur = (uint8_t)gpio_pin_get(g_gpio0, REED_PIN);
            err = k_msgq_put(&g_ble_msgq, &cur, K_NO_WAIT);
            if (0 != err) { LOG_WRN("[BATT] msgq full, drop (err=%d)", err); }
        }

        if (++stats_tick >= STATS_INTERVAL_TICKS)
        {
            stats_tick = 0;
            trinity_log_heap_stats();
            trinity_log_task_stats();
        }

        k_sleep(K_MSEC(REED_DEBOUNCE_MS));

        val = gpio_pin_get(g_gpio0, REED_PIN);
        if (0 > val)
        {
            LOG_WRN("[REED] Pin read failed (err=%d)", val);
        }
        else if (val != last_state)
        {
            LOG_INF("[REED] Changed: %d (%s)", val, val ? "OPEN" : "CLOSED");

            trinity_log_event(val ?
                "EVENT: REED_OPEN\n" : "EVENT: REED_CLOSED\n");

            err = gpio_pin_set_dt(&g_led, !val);
            if (0 != err) { LOG_WRN("[REED] LED set failed (err=%d)", err); }

            state = (uint8_t)val;
            err = k_msgq_put(&g_ble_msgq, &state, K_NO_WAIT);
            if (0 != err) { LOG_WRN("[REED] msgq full, drop (err=%d)", err); }

            last_state = val;
        }
    }
}

/*----------------------------------------------------------------------------*/

int main(void)
{
    int      err          = 0;
    int      initial      = 0;
    uint8_t  init_state   = 0;
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

    LOG_INF("=== ReedSensor Boot ===");
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

    err = gpio_init();
    if (0 != err) { return err; }
    trinity_wdt_kick();

    if (0 != battery_init()) { LOG_WRN("Battery init failed, SOC will read 0"); }
    trinity_wdt_kick();

    /* Read battery before BLE -- VDD stable, radio idle.
     * bt_enable() surge collapses CR2032 rail and corrupts SAADC. */
    soc = battery_read_soc();
    ble_adv_set_batt(soc);
    LOG_INF("Pre-BLE battery mV: %d", battery_read_mv());
    LOG_INF("Pre-BLE battery SOC: %d%%", soc);
    trinity_wdt_kick();

    k_sleep(K_MSEC(100));

    initial = gpio_pin_get(g_gpio0, REED_PIN);
    if (0 > initial) { LOG_ERR("Reed initial read failed (err=%d)", initial); return initial; }

    LOG_INF("Initial state: %d (%s)", initial, initial ? "OPEN" : "CLOSED");

    err = gpio_pin_set_dt(&g_led, (initial ? 0 : 1));
    if (0 != err) { LOG_WRN("LED set failed (err=%d)", err); }

    init_state = (uint8_t)initial;
    err = k_msgq_put(&g_ble_msgq, &init_state, K_NO_WAIT);
    if (0 != err) { LOG_WRN("Initial state msgq put failed (err=%d)", err); }

    tid = k_thread_create(&ble_thread_data,
                           ble_stack, BLE_STACK_SIZE,
                           ble_thread,
                           NULL, NULL, NULL,
                           BLE_PRIORITY, 0, K_NO_WAIT);
    if (NULL == tid) { LOG_ERR("BLE thread create failed"); return -ENOMEM; }
    k_thread_name_set(tid, "ble_thread");

    trinity_wdt_kick();

    LOG_INF("BLE thread spawned");
    LOG_INF("Monitoring reed switch");

    reed_monitor_loop(initial);

    return 0;
}
