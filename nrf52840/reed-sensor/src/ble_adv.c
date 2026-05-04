/******************************************************************************
 * \file    ble_adv.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   BLE advertising and thread for nRF52840 reed sensor node.
 *
 * \details Slow-interval broadcaster advertisement. Manufacturer data
 *          payload carries lock state (1 byte) and battery SOC (1 byte).
 *
 *          MFG data layout (3 bytes):
 *          [0] = MFG_COMPANY_ID  (0xAB)
 *          [1] = door state      (0=closed, 1=open)
 *          [2] = batt_soc        (0-100%)
 *
 * \note    BLE adv race fix (2026-03-23):
 *          Uses bt_le_adv_update_data() instead of stop/start pattern.
 *          The stop/start pattern raced against the BLE radio ISR
 *          (prepare_cb in lll_adv.c:1041) causing KERNEL_OOPS under
 *          rapid reed toggling. update_data() is atomic -- no race.
 *
 * \note    BLE thread WDT fix (2026-03-23):
 *          ble_thread while(1) uses bounded K_TIMEOUT_ABS_TICKS instead
 *          of K_FOREVER. trinity_wdt_kick() called at top of loop so
 *          WDT is fed even during periods of reed inactivity.
 ******************************************************************************/

#include "ble_adv.h"
#include "main.h"
#include "trinity_log.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(ble_adv, LOG_LEVEL_INF);

#define HEARTBEAT_INTERVAL_SEC  240

K_MSGQ_DEFINE(g_ble_msgq, sizeof(uint8_t), 8, 4);

static uint8_t g_mfg_data[MFG_DATA_SIZE] =
    {MFG_COMPANY_ID, 0x00, 0x00};

static struct bt_data g_adv_data[] =
{
    BT_DATA_BYTES(BT_DATA_FLAGS,
        BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE,
        CONFIG_BT_DEVICE_NAME,
        sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    BT_DATA(BT_DATA_MANUFACTURER_DATA,
        g_mfg_data,
        sizeof(g_mfg_data))
};

static struct bt_le_adv_param g_adv_param =
{
    .id           = BT_ID_DEFAULT,
    .options      = BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
    .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
};

/*----------------------------------------------------------------------------*/

static void heartbeat_handler(struct k_timer *p_timer)
{
    int     ret   = 0;
    uint8_t state = 0;

    (void)p_timer;

    /* Re-broadcast current state on heartbeat tick */
    state = g_mfg_data[1];
    ret = k_msgq_put(&g_ble_msgq, &state, K_NO_WAIT);
    if (0 != ret) { LOG_WRN("[HB] msgq full, state drop (err=%d)", ret); }
    else          { LOG_INF("[HB] Sent"); }
}

K_TIMER_DEFINE(g_heartbeat_timer, heartbeat_handler, NULL);

/*----------------------------------------------------------------------------*/

int ble_broadcast(uint8_t state, uint8_t batt_soc)
{
    int err = 0;

    g_mfg_data[1] = state;
    g_mfg_data[2] = batt_soc;

    err = bt_le_adv_update_data(g_adv_data, ARRAY_SIZE(g_adv_data), NULL, 0);
    if (0 != err)
    {
        LOG_ERR("[BLE] Adv update failed (err=%d)", err);
    }
    else
    {
        LOG_INF("[BLE] Broadcasting state=%d batt=%d%%", state, batt_soc);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

void ble_adv_set_batt(uint8_t batt_soc)
{
    g_mfg_data[2] = batt_soc;
}

/*----------------------------------------------------------------------------*/

void ble_thread(void *p_unused1, void *p_unused2, void *p_unused3)
{
    int      err          = 0;
    uint8_t  state        = 0;
    uint32_t update_count = 0;

    (void)p_unused1;
    (void)p_unused2;
    (void)p_unused3;

    LOG_INF("[BLE] Thread started");

    err = bt_enable(NULL);
    if (0 != err) { LOG_ERR("[BLE] Init failed (err=%d)", err); return; }
    trinity_wdt_kick();

    LOG_INF("[BLE] Bluetooth enabled");

    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        err = settings_load();
        if (0 != err) { LOG_WRN("[BLE] Settings load failed (err=%d)", err); }
        trinity_wdt_kick();
    }

    /* Battery already written into g_mfg_data[2] by main() before
     * bt_enable() -- use pre-populated value. Re-reading here would
     * race against the radio and produce low readings on CR2032. */
    LOG_INF("[BLE] Initial batt=%d%%", g_mfg_data[2]);

    err = bt_le_adv_start(&g_adv_param,
                           g_adv_data,
                           ARRAY_SIZE(g_adv_data),
                           NULL, 0);
    if (0 != err) { LOG_ERR("[BLE] Advertising start failed (err=%d)", err); return; }

    LOG_INF("[BLE] Advertising active | %s", CONFIG_BT_DEVICE_NAME);

    k_timer_start(&g_heartbeat_timer,
                  K_SECONDS(HEARTBEAT_INTERVAL_SEC),
                  K_SECONDS(HEARTBEAT_INTERVAL_SEC));

    while (1)
    {
        trinity_wdt_kick();

        (void)k_msgq_get(&g_ble_msgq, &state,
                          K_TIMEOUT_ABS_TICKS(
                             k_uptime_ticks() +
                             k_ms_to_ticks_ceil32(REED_POLL_MS)));

        trinity_wdt_kick();

        update_count++;
        LOG_INF("[BLE] Update #%u state=%d (%s) batt=%d%%",
                update_count, state,
                state ? "OPEN" : "CLOSED",
                g_mfg_data[2]);

        (void)ble_broadcast(state, g_mfg_data[2]);
    }
}
