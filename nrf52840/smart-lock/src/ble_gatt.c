/******************************************************************************
 * \file    ble_gatt.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   BLE GATT service and advertising for nRF52840 smart lock node.
 *
 * \details Connectable BLE peripheral. Exposes lock control, heartbeat,
 *          and battery level characteristics. Advertisement carries
 *          manufacturer data with lock state and battery SOC for passive
 *          hub BLE scanning.
 *
 *          GATT layout:
 *          Service  0x1234
 *            Char   0x1235  lock control   R/W/WWR/Notify
 *            CCC
 *            Char   0x1236  heartbeat      R/Notify
 *            CCC
 *            Char   0x2A19  battery level  R/Notify
 *            CCC
 *
 *          Lock state machine (lock_state.h):
 *          lock_write() validates commands via lock_state_transition().
 *          Commands are rejected while motor is moving (UNLOCKING/LOCKING).
 *          motor.c calls ble_lock_settle() when motor_off_timer fires —
 *          advances UNLOCKING->UNLOCKED or LOCKING->LOCKED, notifies BLE,
 *          updates adv, and saves settled state to settings.
 *
 * \note    Flash concurrency fix (2026-03-24):
 *          lock_write() calls save_lock_state() which wraps settings_save_one()
 *          with trinity_flash_lock/unlock() to prevent concurrent NVMC
 *          access with trinity_log_event() on sysworkq.
 *
 * \note    BT RX WQ stack fix (2026-03-24):
 *          lock_write() submits motor_drive() via work item -- never
 *          blocks BT RX WQ. k_sleep removed from this path entirely.
 *
 * \note    Definition order (circular dependency):
 *          BT_GATT_SERVICE_DEFINE and the GATT handlers cannot be naively
 *          reordered to resolve the forward-reference problem because the
 *          dependency is circular: the service definition requires the handler
 *          addresses in its static struct initializer, while lock_write()
 *          requires smartlock_svc to already be declared so it can take
 *          &smartlock_svc.attrs[2]. Whichever is placed first, the other is
 *          not yet in scope. BT_GATT_SERVICE_DEFINE both declares and defines
 *          the symbol in one macro expansion with no forward-declaration form,
 *          so the only lever available is forward-declaring the handlers above
 *          the service definition. A static function pointer in an initializer
 *          only requires a prior declaration, not a full definition.
 ******************************************************************************/

#include "ble_gatt.h"
#include "motor.h"
#include "config.h"
#include "battery.h"
#include "trinity_log.h"
#include "lock_state.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(ble_gatt, LOG_LEVEL_DBG);

/** \brief Lock state machine — authoritative lock state for this node */
static LOCK_STATE_E g_lock_state    = LOCK_STATE_LOCKED;
static uint8_t      lock_batt_soc   = 0;
static uint8_t      heartbeat_val   = 1;
static uint64_t     last_lock_activity = 0;

static uint8_t lock_mfg_data[MFG_DATA_SIZE] = {MFG_COMPANY_ID, 0x00, 0x00};

static struct bt_data adv_data[] =
{
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE,
            CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x34, 0x12),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, lock_mfg_data, sizeof(lock_mfg_data)),
};

/*----------------------------------------------------------------------------*/
/* Forward declarations                                                        */

static ssize_t lock_read   (struct bt_conn *, const struct bt_gatt_attr *,
                             void *, uint16_t, uint16_t);
static ssize_t lock_write  (struct bt_conn *, const struct bt_gatt_attr *,
                             const void *, uint16_t, uint16_t, uint8_t);
static ssize_t lock_hb_read(struct bt_conn *, const struct bt_gatt_attr *,
                             void *, uint16_t, uint16_t);
static ssize_t batt_read   (struct bt_conn *, const struct bt_gatt_attr *,
                             void *, uint16_t, uint16_t);
static void    hb_ccc_changed(const struct bt_gatt_attr *, uint16_t);

BT_GATT_SERVICE_DEFINE(smartlock_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0x1234)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x1235),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |
        BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        lock_read, lock_write, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x1236),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        lock_hb_read, NULL, &heartbeat_val),
    BT_GATT_CCC(hb_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x2A19),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        batt_read, NULL, &lock_batt_soc),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/*----------------------------------------------------------------------------*/

static ssize_t lock_read(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len, uint16_t offset)
{
    uint8_t ble_val = lock_state_to_ble(g_lock_state);

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &ble_val, sizeof(ble_val));
}

static ssize_t lock_hb_read(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &heartbeat_val, sizeof(heartbeat_val));
}

static ssize_t batt_read(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &lock_batt_soc, sizeof(lock_batt_soc));
}

/******************************************************************************
 * \brief GATT write handler for lock control characteristic (0x1235).
 *
 * \details Routes command through lock state machine. Rejects write if
 *          motor is currently moving (UNLOCKING or LOCKING state).
 *          On valid transition: logs event via Trinity, drives motor,
 *          updates BLE adv with transitional state. Settled state is
 *          written to settings only after motor_off_timer fires via
 *          ble_lock_settle().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static ssize_t lock_write(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           const void *buf, uint16_t len,
                           uint16_t offset, uint8_t flags)
{
    uint8_t      val       = 0;           /**< requested lock value */
    LOCK_STATE_E old_state = LOCK_STATE_LOCKED; /**< previous state */
    LOCK_STATE_E new_state = LOCK_STATE_LOCKED; /**< new state */
    const char  *p_ev      = NULL;        /**< trinity event string */

    if (LOCK_WRITE_LEN != len)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    val       = ((const uint8_t *)buf)[0];
    old_state = g_lock_state;
    new_state = lock_state_transition(old_state, val);

    if (new_state == old_state)
    {
        if (lock_state_is_busy(old_state))
        {
            printk("[LOCK] Command rejected — motor moving (%s)\n",
                   lock_state_label(old_state));
            trinity_log_event("EVENT: LOCK_CMD_REJECTED\n");
        }
        else
        {
            printk("[LOCK] Command ignored — already in state %s\n",
                   lock_state_label(old_state));
        }
        return len;
    }

    g_lock_state       = new_state;
    last_lock_activity = k_uptime_get();

    p_ev = lock_state_event_str(new_state);
    if (NULL != p_ev)
    {
        trinity_log_event(p_ev);
    }

    printk("[LOCK] %s -> %s (cmd=%d)\n",
           lock_state_label(old_state),
           lock_state_label(new_state),
           val);

    /* Update adv with transitional state — settled adv done in ble_lock_settle() */
    ble_adv_update();

    /* Notify connected client of transitional state */
    uint8_t ble_val = lock_state_to_ble(new_state);
    bt_gatt_notify(NULL, &smartlock_svc.attrs[2], &ble_val, sizeof(ble_val));

    /* Drive motor — settle callback fires when motor_off_timer expires */
    motor_drive(val);

    return len;
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Called by motor.c when motor_off_timer fires — settle lock state.
 *
 * \return void
 *
 * \details Advances UNLOCKING->UNLOCKED or LOCKING->LOCKED. Saves settled
 *          state to settings, notifies BLE client, updates adv.
 *          Called from timer ISR context via k_work — safe for settings
 *          and BLE notify.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_lock_settle(void)
{
    LOCK_STATE_E old_state = g_lock_state;               /**< pre-settle state */
    LOCK_STATE_E new_state = lock_state_settle(old_state); /**< settled state */
    const char  *p_ev      = NULL;                        /**< trinity event */
    uint8_t      ble_val   = 0;                           /**< BLE raw value */

    if (new_state == old_state)
    {
        return;
    }

    g_lock_state = new_state;
    ble_val      = lock_state_to_ble(new_state);

    p_ev = lock_state_event_str(new_state);
    if (NULL != p_ev)
    {
        trinity_log_event(p_ev);
    }

    printk("[LOCK] Settled: %s -> %s\n",
           lock_state_label(old_state),
           lock_state_label(new_state));

    /* Save settled state to persistent settings */
    trinity_flash_lock();
    settings_save_one("lock/state", &ble_val, sizeof(ble_val));
    trinity_flash_unlock();

    /* Notify BLE client and update adv with settled state */
    bt_gatt_notify(NULL, &smartlock_svc.attrs[2], &ble_val, sizeof(ble_val));
    ble_adv_update();
}

/*----------------------------------------------------------------------------*/

static void hb_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    printk("[HB] CCC changed: %d\n", value);
}

/*----------------------------------------------------------------------------*/

void ble_adv_update(void)
{
    uint8_t ble_val = lock_state_to_ble(g_lock_state);

    lock_mfg_data[MFG_LOCK_STATE_IDX] = ble_val;
    lock_mfg_data[MFG_BATT_IDX]       = lock_batt_soc;
    bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
}

void ble_set_batt(uint8_t soc)
{
    lock_batt_soc = soc;
    bt_gatt_notify(NULL, &smartlock_svc.attrs[8],
                   &lock_batt_soc, sizeof(lock_batt_soc));
    ble_adv_update();
}

void ble_set_lock_state(uint8_t state)
{
    g_lock_state = (LOCK_STATE_E)state;
    ble_adv_update();
}

/*----------------------------------------------------------------------------*/

static struct k_work g_adv_restart_work;
static struct k_work g_hb_work;
static struct k_work g_batt_work;
static struct k_work g_stats_work;

static void adv_restart_handler(struct k_work *work)
{
    static const struct bt_le_adv_param param =
    {
        .id           = BT_ID_DEFAULT,
        .options      = BT_LE_ADV_OPT_CONN,
        .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
        .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
    };

    int err = bt_le_adv_start(&param, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (0 != err) { printk("[BLE] Adv restart failed: %d\n", err); }
    else          { printk("[BLE] Advertising restarted\n"); }
}

static void hb_work_handler(struct k_work *work)
{
    uint64_t now = k_uptime_get();

    if ((now - last_lock_activity) > (IDLE_HEARTBEAT_SEC * 1000ULL))
    {
        int err = bt_gatt_notify(NULL, &smartlock_svc.attrs[5],
                                 &heartbeat_val, sizeof(heartbeat_val));
        if (err && err != -ENOTCONN) { printk("[HB] Notify failed: %d\n", err); }
        else                         { printk("[HB] Sent\n"); }
        ble_adv_update();
    }
}

static void batt_work_handler(struct k_work *work)
{
    int mv = battery_read_mv();
    if (0 > mv) { printk("[BATT] Read failed\n"); return; }

    uint8_t soc = mv_to_soc(mv);
    printk("[BATT] raw mv=%d soc=%d%%\n", mv, soc);
    ble_set_batt(soc);
}

static void stats_work_handler(struct k_work *work)
{
    trinity_wdt_kick();
    trinity_log_heap_stats();
    trinity_log_task_stats();
}

static void idle_hb_handler(struct k_timer *t)  { k_work_submit(&g_hb_work);    }
static void batt_handler(struct k_timer *t)      { k_work_submit(&g_batt_work);  }
static void stats_handler(struct k_timer *t)     { k_work_submit(&g_stats_work); }

K_TIMER_DEFINE(g_idle_hb_timer, idle_hb_handler, NULL);
K_TIMER_DEFINE(g_batt_timer,    batt_handler,     NULL);
K_TIMER_DEFINE(g_stats_timer,   stats_handler,    NULL);

/*----------------------------------------------------------------------------*/

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (0 != err) { printk("[BLE] Connect failed: %d\n", err); return; }
    printk("[BLE] Connected\n");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("[BLE] Disconnected (reason %d)\n", reason);
    k_work_submit(&g_adv_restart_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) =
{
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

/*----------------------------------------------------------------------------*/

int ble_gatt_init(void)
{
    int err = 0;

    static const struct bt_le_adv_param slow_adv_param =
    {
        .id           = BT_ID_DEFAULT,
        .options      = BT_LE_ADV_OPT_CONN,
        .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
        .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
    };

    err = bt_enable(NULL);
    if (0 != err) { LOG_ERR("BLE init failed"); return err; }
    trinity_wdt_kick();
    LOG_INF("BLE enabled");
    trinity_log_event("EVENT: BLE_READY\n");

    err = bt_le_adv_start(&slow_adv_param,
                           adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (0 != err) { LOG_ERR("BLE advertising failed"); return err; }

    trinity_log_event("EVENT: BLE_ADV_START\n");
    ble_adv_update();

    k_work_init(&g_adv_restart_work, adv_restart_handler);
    k_work_init(&g_hb_work,          hb_work_handler);
    k_work_init(&g_batt_work,        batt_work_handler);
    k_work_init(&g_stats_work,       stats_work_handler);

    last_lock_activity = k_uptime_get();

    k_timer_start(&g_idle_hb_timer,
                  K_SECONDS(IDLE_HEARTBEAT_SEC),
                  K_SECONDS(IDLE_HEARTBEAT_SEC));
    k_timer_start(&g_batt_timer,
                  K_SECONDS(BATT_UPDATE_SEC),
                  K_SECONDS(BATT_UPDATE_SEC));
    k_timer_start(&g_stats_timer,
                  K_SECONDS(STATS_INTERVAL_SEC),
                  K_SECONDS(STATS_INTERVAL_SEC));

    return 0;
}
