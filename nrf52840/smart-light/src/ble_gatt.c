/******************************************************************************
 * \file    ble_gatt.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   BLE GATT service and advertising for nRF52840 smart light node.
 *
 * \details Connectable BLE peripheral. Exposes a light control
 *          characteristic (read/write) and a heartbeat characteristic
 *          (read/notify). Advertisement carries manufacturer data with
 *          current light state for passive hub BLE scanning.
 *
 *          GATT layout:
 *          Service  0xABCD
 *            Char   0xAB01  light control  R/W/WWR
 *            Char   0xAB02  heartbeat      R/Notify
 *            CCC
 *
 * \note    adv restart fix (2026-03-24):
 *          restart_advertising() uses a delayable work item on retry so
 *          sysworkq is never blocked for 1s on the retry path.
 *
 * \note    Flash concurrency fix (2026-03-24):
 *          write_light_control() calls trinity_log_event() which acquires
 *          the flash mutex internally -- no external lock needed.
 *
 * \note    settings_load() fix (2026-03-24):
 *          Called after bt_enable() so BLE stack has a valid identity
 *          address and bt_le_adv_start() does not hang.
 ******************************************************************************/

#include "ble_gatt.h"
#include "main.h"
#include "trinity_log.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#define BT_UUID_LIGHT_SERVICE   BT_UUID_DECLARE_16(0xABCD)
#define BT_UUID_LIGHT_CONTROL   BT_UUID_DECLARE_16(0xAB01)

static uint8_t  light_state        = 0;
static uint8_t  heartbeat_val      = 1;
static uint64_t last_activity_time = 0;

static const struct bt_gatt_attr *heartbeat_attr = NULL;
static struct bt_conn            *current_conn   = NULL;

static uint8_t light_mfg_data[MFG_DATA_SIZE] = {MFG_COMPANY_ID, 0x00};

static struct bt_data adv_data[] =
{
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, 'L','i','g','h','t','N','F'),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, light_mfg_data, sizeof(light_mfg_data)),
};

/*----------------------------------------------------------------------------*/

void ble_adv_update(void)
{
    light_mfg_data[MFG_STATE_IDX] = light_state;
    bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
}

/*----------------------------------------------------------------------------*/

static struct k_work_delayable adv_restart_dwork;

static void restart_advertising(struct k_work *work)
{
    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
                               adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (0 != err)
    {
        printk("[BLE] Adv restart failed: %d, retrying in 1s\n", err);
        k_work_reschedule(&adv_restart_dwork, K_MSEC(1000));
    }
    else
    {
        printk("[BLE] Advertising restarted\n");
    }
}

/*----------------------------------------------------------------------------*/

static struct k_work hb_work;

static void hb_work_handler(struct k_work *work)
{
    uint64_t now = k_uptime_get();

    if ((now - last_activity_time) > (IDLE_HEARTBEAT_SEC * 1000ULL))
    {
        ble_adv_update();
        if (current_conn && heartbeat_attr)
        {
            heartbeat_val = 1;
            int err = bt_gatt_notify(current_conn, heartbeat_attr,
                                     &heartbeat_val, sizeof(heartbeat_val));
            if (err && err != -ENOTCONN)
            {
                printk("[HB] Notify failed: %d\n", err);
            }
        }
        last_activity_time = k_uptime_get();
        printk("[HB] Sent\n");
    }
}

static void idle_hb_handler(struct k_timer *timer) { k_work_submit(&hb_work); }
K_TIMER_DEFINE(idle_hb_timer, idle_hb_handler, NULL);

/*----------------------------------------------------------------------------*/

static struct k_work stats_work;

static void stats_work_handler(struct k_work *work)
{
    trinity_wdt_kick();
    trinity_log_heap_stats();
    trinity_log_task_stats();
}

static void stats_timer_handler(struct k_timer *timer) { k_work_submit(&stats_work); }
K_TIMER_DEFINE(stats_timer, stats_timer_handler, NULL);

/*----------------------------------------------------------------------------*/

static ssize_t write_light_control(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len,
                                    uint16_t offset, uint8_t flags)
{
    uint8_t new_state = 0;

    if (LIGHT_WRITE_LEN != len)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    new_state = *((const uint8_t *)buf);

    if (new_state > LIGHT_STATE_MAX)
    {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    light_state        = new_state;
    last_activity_time = k_uptime_get();

    relay_set(light_state);
    ble_adv_update();

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

/*----------------------------------------------------------------------------*/

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_le_conn_param param =
    {
        .interval_min = 24, .interval_max = 40,
        .latency = 0, .timeout = 400
    };

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (0 != err)
    {
        printk("[BLE] Connection failed to %s: %d\n", addr, err);
        return;
    }

    if (current_conn && current_conn != conn)
    {
        bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(current_conn);
    }

    current_conn = bt_conn_ref(conn);
    printk("[BLE] Connected to %s\n", addr);
    trinity_log_event("EVENT: BLE_CONNECTED\n");

    bt_conn_le_param_update(conn, &param);
    last_activity_time = k_uptime_get();
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("[BLE] Disconnected from %s (reason: %d)\n", addr, reason);
    trinity_log_event("EVENT: BLE_DISCONNECTED\n");

    if (current_conn == conn)
    {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    k_work_reschedule(&adv_restart_dwork, K_NO_WAIT);
}

BT_CONN_CB_DEFINE(conn_callbacks) =
{
    .connected    = connected,
    .disconnected = disconnected,
};

/*----------------------------------------------------------------------------*/

int ble_gatt_init(void)
{
    int err = 0;

    err = bt_enable(NULL);
    if (0 != err) { printk("[BLE] bt_enable failed (%d)\n", err); return err; }
    trinity_wdt_kick();
    printk("[BLE] Bluetooth enabled\n");

    err = settings_load();
    if (0 != err) { printk("[BLE] Settings load failed (%d)\n", err); }
    trinity_wdt_kick();

    heartbeat_attr = &light_svc.attrs[4];
    k_work_init_delayable(&adv_restart_dwork, restart_advertising);
    k_work_init(&hb_work, hb_work_handler);
    k_work_init(&stats_work, stats_work_handler);

    ble_adv_update();

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
                           adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (0 != err) { printk("[BLE] Adv start failed (%d)\n", err); return err; }

    printk("[BLE] Advertising active\n");
    trinity_log_event("EVENT: BLE_ADV_START\n");

    last_activity_time = k_uptime_get();

    k_timer_start(&idle_hb_timer,
                  K_SECONDS(IDLE_HEARTBEAT_SEC),
                  K_SECONDS(IDLE_HEARTBEAT_SEC));
    k_timer_start(&stats_timer,
                  K_SECONDS(STATS_INTERVAL_SEC),
                  K_SECONDS(STATS_INTERVAL_SEC));

    return 0;
}
