#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>
#include <string.h>
#include <nrf.h>
#include "trinity_log.h"

/*
 * \note settings_load() fix (2026-03-24):
 *       bt_hci_core logs "No ID address. App must call settings_load()"
 *       on every boot. Without it the BLE stack has no identity address
 *       and bt_le_adv_start() hangs. Fix: call settings_load() after
 *       bt_enable(), same as smart-lock and reed-sensor.
 *
 * \note Init stage cookie (2026-03-24):
 *       g_init_stage breadcrumbs written before each risky init call.
 *       Printed alongside PRE_INIT_CRASH to narrow crash to one call.
 *
 * \note WDT boot fix (2026-03-24):
 *       trinity_wdt_kick() added after bt_enable() and settings_load()
 *       to prevent WDT firing during boot in field mode.
 *
 * \note Flash concurrency fix (2026-03-24):
 *       Root cause of crash confirmed via register decode:
 *         PC = 0x5954494E ("NITY" ASCII) -- return address overwritten
 *         SP = 0x003BA538 -- stack pointer blown into flash address space
 *       trinity_thread_cb() had static char msg[] -- shared across all
 *       thread_analyzer callbacks, not thread-safe. Combined with no
 *       flash mutex, concurrent write_entry() calls from BT RX WQ
 *       (write_light_control -> trinity_log_event) and sysworkq
 *       (stats_work -> trinity_thread_cb -> trinity_log_event) corrupted
 *       the static buffer and overwrote a return address with "NITY".
 *       Fix: trinity_log.c now has g_flash_mutex in write_entry() and
 *       local (not static) msg buffer in trinity_thread_cb(). Callers
 *       that write flash outside Trinity use trinity_flash_lock/unlock().
 *       write_light_control() calls trinity_log_event() which acquires
 *       the mutex internally -- no external lock needed there.
 *
 * \note adv restart fix (2026-03-24):
 *       restart_advertising() had k_sleep(K_MSEC(1000)) on retry path,
 *       blocking sysworkq for 1s. Replaced with a delayable work item
 *       so sysworkq is never blocked.
 *
 * \note RESETREAS clear fix (2026-03-27):
 *       NRF_POWER->RESETREAS is a latched OR-history register -- it is NOT
 *       cleared automatically on reset. Read at the very top of main(),
 *       then immediately write 0xFFFFFFFF to clear all latched bits.
 *       Passed to trinity_log_boot_reason() after trinity_log_init() --
 *       Trinity handles decode and logging.
 *
 * \note RESETREAS raw hex logging (2026-03-27):
 *       trinity_log_boot_reason() logs both decoded label and raw hex.
 *       decode_reset_reason() and NRF_RESET_* defines removed from main.c
 *       and centralised in trinity_log.c -- all nRF52840 boards share one
 *       implementation with identical output format.
 */

#include "main.h"

static uint8_t heartbeat_val = 1;
static uint64_t last_activity_time = 0;
static const struct bt_gatt_attr *heartbeat_attr = NULL;

#define RELAY_PORT DT_NODELABEL(gpio0)
#define RELAY_PIN 11
#define LIGHT_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec light = GPIO_DT_SPEC_GET(LIGHT_NODE, gpios);

#define BT_UUID_LIGHT_SERVICE   BT_UUID_DECLARE_16(0xABCD)
#define BT_UUID_LIGHT_CONTROL   BT_UUID_DECLARE_16(0xAB01)

static uint8_t light_state = 0;
static const struct device *relay_dev;
static struct bt_conn *current_conn = NULL;

static uint8_t light_mfg_data[2] = {0xAD, 0x00};

static struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, 'L','i','g','h','t','N','F'),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, light_mfg_data, sizeof(light_mfg_data)),
};

static void update_adv_data(void)
{
    light_mfg_data[1] = light_state;
    bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
}

static struct k_work_delayable adv_restart_dwork;

static void restart_advertising(struct k_work *work)
{
    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
                              adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (err) {
        printk("[BLE] Adv restart failed: %d, retrying in 1s\n", err);
        k_work_reschedule(&adv_restart_dwork, K_MSEC(1000));
    } else {
        printk("[BLE] Advertising restarted\n");
    }
}

static struct k_work hb_work;

static void hb_work_handler(struct k_work *work)
{
    uint64_t now = k_uptime_get();
    if ((now - last_activity_time) > (IDLE_HEARTBEAT_SEC * 1000ULL)) {
        update_adv_data();
        if (current_conn && heartbeat_attr) {
            heartbeat_val = 1;
            int err = bt_gatt_notify(current_conn, heartbeat_attr,
                                     &heartbeat_val, sizeof(heartbeat_val));
            if (err && err != -ENOTCONN) printk("[HB] Notify failed: %d\n", err);
        }
        last_activity_time = k_uptime_get();
        printk("[HB] Sent\n");
    }
}

static void idle_hb_handler(struct k_timer *timer) { k_work_submit(&hb_work); }
K_TIMER_DEFINE(idle_hb_timer, idle_hb_handler, NULL);

static struct k_work stats_work;

static void stats_work_handler(struct k_work *work)
{
    trinity_wdt_kick();
    trinity_log_heap_stats();
    trinity_log_task_stats();
}

static void stats_timer_handler(struct k_timer *timer) { k_work_submit(&stats_work); }
K_TIMER_DEFINE(stats_timer, stats_timer_handler, NULL);

static ssize_t write_light_control(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len,
                                   uint16_t offset, uint8_t flags)
{
    if (len != 1) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    uint8_t new_state = *((uint8_t *)buf);
    if (new_state > 1) return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);

    light_state = new_state;
    last_activity_time = k_uptime_get();

    gpio_pin_set(relay_dev, RELAY_PIN, light_state);
    gpio_pin_set_dt(&light, light_state);
    update_adv_data();

    printk("[LIGHT] State changed to: %s\n", light_state ? "ON" : "OFF");
    trinity_log_event(light_state ? "EVENT: LIGHT_ON\n" : "EVENT: LIGHT_OFF\n");
    return len;
}

static ssize_t read_light_control(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &light_state, sizeof(light_state));
}

static ssize_t read_heartbeat(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &heartbeat_val, sizeof(heartbeat_val));
}

BT_GATT_SERVICE_DEFINE(light_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_LIGHT_SERVICE),
    BT_GATT_CHARACTERISTIC(
        BT_UUID_LIGHT_CONTROL,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_light_control, write_light_control, &light_state),
    BT_GATT_CHARACTERISTIC(
        BT_UUID_DECLARE_16(0xAB02),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        read_heartbeat, NULL, &heartbeat_val),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    if (err) { printk("[BLE] Connection failed to %s: %d\n", addr, err); return; }
    if (current_conn && current_conn != conn) {
        bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(current_conn);
    }
    current_conn = bt_conn_ref(conn);
    printk("[BLE] Connected to %s\n", addr);
    trinity_log_event("EVENT: BLE_CONNECTED\n");
    struct bt_le_conn_param param = {
        .interval_min = 24, .interval_max = 40,
        .latency = 0, .timeout = 400
    };
    bt_conn_le_param_update(conn, &param);
    last_activity_time = k_uptime_get();
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("[BLE] Disconnected from %s (reason: %d)\n", addr, reason);
    trinity_log_event("EVENT: BLE_DISCONNECTED\n");
    if (current_conn == conn) { bt_conn_unref(current_conn); current_conn = NULL; }
    k_work_reschedule(&adv_restart_dwork, K_NO_WAIT);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

int main(void)
{
    int err;

    uint32_t reset_reason = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS  = 0xFFFFFFFF;

#if !defined(CONFIG_TRINITY_MODE_BENCH)
    g_init_stage = TRINITY_STAGE_WDT_INIT;
    trinity_wdt_init();
    trinity_wdt_kick();
#endif

    printk("\n=== LightNF BLE Relay Controller ===\n");

    trinity_log_dump_previous();

    g_init_stage = TRINITY_STAGE_GPIO;

    relay_dev = DEVICE_DT_GET(RELAY_PORT);
    if (!device_is_ready(relay_dev)) return -1;
    if (gpio_pin_configure(relay_dev, RELAY_PIN, GPIO_OUTPUT_INACTIVE)) return -1;

    if (!device_is_ready(light.port)) return -1;
    if (gpio_pin_configure_dt(&light, GPIO_OUTPUT_INACTIVE)) return -1;
    printk("[GPIO] Light/Relay initialized (OFF)\n");

    g_init_stage = TRINITY_STAGE_LOG_INIT;

    if (trinity_log_init() == 0) {
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

    err = bt_enable(NULL);
    if (err) return -1;
    trinity_wdt_kick();
    printk("[BLE] Bluetooth enabled\n");

    err = settings_load();
    if (err) { printk("[BLE] Settings load failed (%d)\n", err); }
    trinity_wdt_kick();

    g_init_stage = TRINITY_STAGE_BT_ADV;

    heartbeat_attr = &light_svc.attrs[4];
    k_work_init_delayable(&adv_restart_dwork, restart_advertising);
    k_work_init(&hb_work, hb_work_handler);
    k_work_init(&stats_work, stats_work_handler);

    update_adv_data();
    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
                          adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (err) return -1;
    printk("[BLE] Advertising active\n");
    trinity_log_event("EVENT: BLE_ADV_START\n");

    last_activity_time = k_uptime_get();
    k_timer_start(&idle_hb_timer,
                  K_SECONDS(IDLE_HEARTBEAT_SEC),
                  K_SECONDS(IDLE_HEARTBEAT_SEC));
    k_timer_start(&stats_timer,
                  K_SECONDS(STATS_INTERVAL_SEC),
                  K_SECONDS(STATS_INTERVAL_SEC));

    g_init_stage = TRINITY_STAGE_MAIN_LOOP;

    while (1) {
        trinity_wdt_kick();
        k_sleep(K_MSEC(2000));
    }

    return 0;
}
