/******************************************************************************
 * \file    ble_adv.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   BLE advertising for ESP32-C3 PIR motion sensor node.
 *
 * \details Slow-interval broadcaster advertisement. Manufacturer data
 *          payload carries motion count (4 bytes), battery SOC (1 byte),
 *          and occupied flag (1 byte).
 *
 *          MFG data layout (8 bytes):
 *          [0]   = 0xFF  (company ID low  -- no registered ID, sentinel)
 *          [1]   = 0xFF  (company ID high)
 *          [2]   = motion_count >> 24
 *          [3]   = motion_count >> 16
 *          [4]   = motion_count >>  8
 *          [5]   = motion_count & 0xFF
 *          [6]   = batt_soc (0-100%)
 *          [7]   = occupied (0=empty, 1=occupied)
 *
 * \note    BT_SMP disabled (2026-03-23):
 *          CONFIG_BT_SMP=n -- PIR is broadcaster-only, no pairing needed.
 *          Null SMP callbacks are a KERNEL_OOPS vector if a phone attempts
 *          pairing. Matches the fix applied to reed-sensor.
 ******************************************************************************/

#include "ble_adv.h"
#include "main.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_adv, LOG_LEVEL_INF);

static uint8_t mfg_data[MFG_DATA_SIZE] = {
    MFG_COMPANY_ID_0,
    MFG_COMPANY_ID_1,
    0x00, 0x00, 0x00, 0x00,  /* motion count */
    0x00,                     /* batt soc     */
    0x00                      /* occupied     */
};

static struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data))
};

static const struct bt_le_adv_param adv_param = {
    .id           = BT_ID_DEFAULT,
    .options      = BT_LE_ADV_OPT_NONE,
    .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
    .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
};

/*----------------------------------------------------------------------------*/

static void print_mac(void)
{
    bt_addr_le_t id_addr;
    size_t       count = 1;
    char         addr_str[BT_ADDR_LE_STR_LEN];

    bt_id_get(&id_addr, &count);
    if (0 == count) { LOG_WRN("No BLE identity found"); return; }

    bt_addr_le_to_str(&id_addr, addr_str, sizeof(addr_str));
    LOG_INF("BLE MAC: %s", addr_str);
}

/*----------------------------------------------------------------------------*/

int ble_adv_init(void)
{
    int ret = bt_enable(NULL);
    if (0 != ret)
    {
        LOG_ERR("BLE init failed (%d)", ret);
        return ret;
    }

    LOG_INF("BLE initialized");

    ret = bt_le_adv_start(&adv_param, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    if (0 != ret)
    {
        LOG_ERR("Advertising failed (%d)", ret);
        return ret;
    }

    LOG_INF("Advertising started");
    print_mac();

    return 0;
}

/*----------------------------------------------------------------------------*/

void ble_adv_update(uint32_t motion_count, uint8_t batt_soc, uint8_t occupied)
{
    pack_motion_count(mfg_data, motion_count);
    mfg_data[MFG_BATT_IDX]     = batt_soc;
    mfg_data[MFG_OCCUPIED_IDX] = occupied;
    bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
}
