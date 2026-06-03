/******************************************************************************
 * \file    ble_adv.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    2026-06-02
 *
 * \brief   BLE advertising and thread for nRF52840 temp sensor node.
 *
 * \details Slow-interval broadcaster advertisement. Manufacturer data
 *          payload carries temperature in tenths of °C (int16, 2 bytes
 *          little-endian) and battery SOC (1 byte).
 *
 *          MFG data layout (4 bytes):
 *          [0] = MFG_COMPANY_ID  (0xAE)
 *          [1] = temp_decidegc low byte
 *          [2] = temp_decidegc high byte
 *          [3] = batt_soc        (0-100%)
 *
 * \note    Inherited from reed-sensor:
 *          bt_le_adv_update_data() used instead of stop/start to avoid
 *          prepare_cb race (lll_adv.c:1041) under rapid updates.
 *          ble_thread uses K_TIMEOUT_ABS_TICKS (not K_FOREVER) so WDT
 *          is fed even during quiet periods.
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

/* Queue carries int16_t values (2 bytes each), 8 slots, 2-byte alignment */
K_MSGQ_DEFINE(g_ble_msgq, sizeof(int16_t), 8, 2);

static uint8_t g_mfg_data[MFG_DATA_SIZE] =
    {MFG_COMPANY_ID, 0x00, 0x00, 0x00};

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
    int16_t temp = 0;
    int     ret  = 0;

    (void)p_timer;

    /* Re-broadcast current temperature on heartbeat tick */
    temp = (int16_t)((g_mfg_data[2] << 8) | g_mfg_data[1]);
    ret = k_msgq_put(&g_ble_msgq, &temp, K_NO_WAIT);
    if (0 != ret) { LOG_WRN("[HB] msgq full, temp drop (err=%d)", ret); }
    else          { LOG_INF("[HB] Sent"); }
}

K_TIMER_DEFINE(g_heartbeat_timer, heartbeat_handler, NULL);

/*----------------------------------------------------------------------------*/

int ble_broadcast(int16_t temp_decidegc, uint8_t batt_soc)
{
    int err = 0;

    /* Store int16 little-endian in bytes [1] and [2] */
    g_mfg_data[1] = (uint8_t)(temp_decidegc & 0xFF);
    g_mfg_data[2] = (uint8_t)((temp_decidegc >> 8) & 0xFF);
    g_mfg_data[3] = batt_soc;

    err = bt_le_adv_update_data(g_adv_data, ARRAY_SIZE(g_adv_data), NULL, 0);
    if (0 != err)
    {
        LOG_ERR("[BLE] Adv update failed (err=%d)", err);
    }
    else
    {
        LOG_INF("[BLE] Broadcasting temp=%d.%d°C batt=%d%%",
                (int)(temp_decidegc / 10),
                (int)(temp_decidegc < 0 ?
                      -(temp_decidegc % 10) : temp_decidegc % 10),
                batt_soc);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

void ble_adv_set_batt(uint8_t batt_soc)
{
    g_mfg_data[3] = batt_soc;
}

/*----------------------------------------------------------------------------*/

void ble_thread(void *p_unused1, void *p_unused2, void *p_unused3)
{
    int      err          = 0;
    int16_t  temp         = 0;
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

    LOG_INF("[BLE] Initial batt=%d%%", g_mfg_data[3]);

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

        (void)k_msgq_get(&g_ble_msgq, &temp,
                          K_TIMEOUT_ABS_TICKS(
                             k_uptime_ticks() +
                             k_ms_to_ticks_ceil32(TEMP_POLL_MS)));

        trinity_wdt_kick();

        update_count++;
        LOG_INF("[BLE] Update #%u temp=%d.%d°C batt=%d%%",
                update_count,
                (int)(temp / 10),
                (int)(temp < 0 ? -(temp % 10) : temp % 10),
                g_mfg_data[3]);

        (void)ble_broadcast(temp, g_mfg_data[3]);
    }
}
