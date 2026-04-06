#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/addr.h>
#include <string.h>
#include "max17048.h"
#include "trinity_log.h"
#include "main.h"
#include <zephyr/drivers/i2c.h>
#include <errno.h>

LOG_MODULE_REGISTER(pir_ble, LOG_LEVEL_INF);

/*
 * \note WDT main-loop fix (2026-03-23):
 *       Previous code: k_sleep(K_SECONDS(10)) THEN trinity_wdt_kick().
 *       WDT timeout is 3s. The sleep fired before the kick — device
 *       reset on every iteration in field mode. Fixed by moving the kick
 *       to the TOP of the loop before the sleep, and reducing sleep to
 *       2s so the pattern matches the other Trinity nodes. The 10s
 *       tick counter scaling is updated to preserve the original
 *       5-minute battery interval (150 ticks) and 60s stats interval
 *       (30 ticks) at the new 2s rate.
 *
 * \note WDT boot fix (2026-03-23):
 *       trinity_wdt_kick() added after trinity_wdt_init() and after
 *       bt_enable() (~1-2s blocking call) to prevent WDT firing during
 *       boot before the main loop starts kicking.
 *
 * \note Pre-init canary (2026-03-23):
 *       trinity_log.c now writes a .noinit canary at PRE_KERNEL_1 before
 *       any SYS_INIT driver runs. trinity_log_dump_previous() reports
 *       PRE_INIT_CRASH if the board died before trinity_log_init().
 *
 * \note BT_SMP disabled (2026-03-23):
 *       CONFIG_BT_SMP=n — PIR sensor is broadcaster-only, no pairing
 *       needed. Null SMP callbacks are a KERNEL_OOPS vector if a phone
 *       attempts pairing. Matches the fix applied to reed-sensor.
 */

#define PIR_PIN              3
#define GPIO_NODE            DT_NODELABEL(gpio0)


static uint32_t motion_count = 0;
static uint8_t  batt_soc     = 0;

static const struct bt_le_adv_param adv_param = {
    .id           = BT_ID_DEFAULT,
    .options      = BT_LE_ADV_OPT_NONE,
    .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
    .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
};

static uint8_t mfg_data[7] = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00 };

static struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data))
};

static struct gpio_callback pir_cb_data;

static void update_adv(void)
{
    mfg_data[2] = (motion_count >> 24) & 0xFF;
    mfg_data[3] = (motion_count >> 16) & 0xFF;
    mfg_data[4] = (motion_count >>  8) & 0xFF;
    mfg_data[5] =  motion_count        & 0xFF;
    mfg_data[6] =  batt_soc;
    bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
}

static void print_advertised_mac(void)
{
    bt_addr_le_t id_addr;
    size_t count = 1;
    bt_id_get(&id_addr, &count);
    if (count == 0) { LOG_WRN("No BLE identity found"); return; }
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&id_addr, addr_str, sizeof(addr_str));
    LOG_INF("BLE MAC: %s", addr_str);
}

void pir_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (gpio_pin_get(dev, PIR_PIN)) {
        motion_count++;
        update_adv();
        trinity_log_event("EVENT: MOTION\n");
        LOG_INF("Motion detected! Count=%u  Batt=%d%%", motion_count, batt_soc);
    }
}

int main(void)
{
    int ret;

    LOG_INF("===========================================");
    LOG_INF("  ESP32-C3 PIR BLE Motion Sensor");
    LOG_INF("===========================================");

    /* ---- Trinity: init flash log ---- */
    trinity_log_dump_previous();
    trinity_log_init();
    trinity_log_event("EVENT: BOOT\n");

    /* ---- Trinity: arm watchdog then kick for fresh 3s boot budget ---- */
    trinity_wdt_init();
    trinity_wdt_kick();

    const struct device *gpio = DEVICE_DT_GET(GPIO_NODE);
    if (!device_is_ready(gpio)) { LOG_ERR("GPIO not ready"); return 0; }
    gpio_pin_configure(gpio, PIR_PIN, GPIO_INPUT);

    /* FIX: kick before bt_enable — it blocks ~1-2s, exhausting the 3s budget */
    trinity_wdt_kick();
    ret = bt_enable(NULL);
    if (ret) { LOG_ERR("BLE init failed (%d)", ret); return 0; }
    trinity_wdt_kick(); /* fresh 3s budget after bt_enable completes */
    LOG_INF("BLE initialized");

    max17048_init();
    batt_soc = (uint8_t)max17048_read_soc();
    int mv   = max17048_read_mv();
    LOG_INF("Battery: %d%% (%d mV)", batt_soc, mv);
    update_adv();

    ret = bt_le_adv_start(&adv_param, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (ret) { LOG_ERR("Advertising failed (%d)", ret); return 0; }
    LOG_INF("Advertising started");
    print_advertised_mac();

    gpio_init_callback(&pir_cb_data, pir_callback, BIT(PIR_PIN));
    gpio_add_callback(gpio, &pir_cb_data);
    gpio_pin_interrupt_configure(gpio, PIR_PIN, GPIO_INT_EDGE_BOTH);
    LOG_INF("PIR interrupt configured");
    LOG_INF("*** System Ready ***");

    int tick = 0, batt_tick = 0, stats_tick = 0;

    while (1) {
        /*
         * FIX: kick BEFORE sleep. Previous order was sleep(10s) then kick —
         * WDT timeout is 3s so the device reset on every loop in field mode.
         * Sleep reduced to 2s (same as reed-sensor) so the kick pattern is
         * consistent across all Trinity nodes. Tick multipliers updated above.
         */
        trinity_wdt_kick();
        k_sleep(K_MSEC(2000));

        if (tick % 30 == 0) {
           LOG_INF("[%d] Motion count: %u  Batt=%d%%", tick++, motion_count, batt_soc);
        }

        /* Battery update every 5 min */
        if (++batt_tick >= BATT_UPDATE_TICKS) {
            batt_tick = 0;
            batt_soc = (uint8_t)max17048_read_soc();
            mv       = max17048_read_mv();
            LOG_INF("Battery updated: %d%% (%d mV)", batt_soc, mv);
            update_adv();
        }

        /* ---- Trinity: heap and task stats every 60s ---- */
        if (++stats_tick >= STATS_INTERVAL_TICKS) {
            stats_tick = 0;
            trinity_log_heap_stats();
            trinity_log_task_stats();
        }
    }

    return 0;
}
