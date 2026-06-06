#ifndef STUB_BLUETOOTH_H
#define STUB_BLUETOOTH_H

/*
 * Stub: zephyr/bluetooth/bluetooth.h
 * ble_adv.c uses bt_enable(), bt_le_adv_start(), bt_le_adv_update_data(),
 * bt_le_adv_stop(), and the BT_DATA / BT_LE_ADV_OPT_* macros.
 *
 * bt_le_adv_update_data() and bt_le_adv_stop() are FFF-mocked in the
 * test file. The rest are stubbed here so ble_adv.c compiles cleanly.
 */

#include <stdint.h>
#include <stddef.h>

/* bt_data: used to build the advertising payload array in ble_adv.c */
struct bt_data {
    uint8_t        type;
    uint8_t        data_len;
    const uint8_t *data;
};

/* bt_le_adv_param: advertising interval / options */
struct bt_le_adv_param {
    uint8_t  id;
    uint32_t options;
    uint16_t interval_min;
    uint16_t interval_max;
};

/* Manifest constants used in ble_adv.c initialisers */
#define BT_ID_DEFAULT              0
#define BT_LE_ADV_OPT_USE_IDENTITY (1U << 0)
#define BT_LE_AD_GENERAL           0x02
#define BT_LE_AD_NO_BREDR          0x04
#define BT_DATA_FLAGS              0x01
#define BT_DATA_NAME_COMPLETE      0x09
#define BT_DATA_MANUFACTURER_DATA  0xFF
#define BT_GAP_ADV_SLOW_INT_MIN    0x0640  /* 1 s */
#define BT_GAP_ADV_SLOW_INT_MAX    0x0780  /* 1.2 s */

/* Convenience constructors matching zephyr/bluetooth/bluetooth.h */
#define BT_DATA_BYTES(type, ...) \
    { (type), sizeof((uint8_t[]){__VA_ARGS__}), (uint8_t[]){__VA_ARGS__} }
#define BT_DATA(type, data_ptr, data_len_val) \
    { (type), (data_len_val), (const uint8_t *)(data_ptr) }

/* CONFIG_BT_DEVICE_NAME -- provide a test name if not set by prj.conf */
#ifndef CONFIG_BT_DEVICE_NAME
#define CONFIG_BT_DEVICE_NAME "ReedTest"
#endif

/* bt_enable: FFF-mockable stub -- returns 0 (success) by default */
static inline int bt_enable(void *cb) { (void)cb; return 0; }

/* bt_le_adv_start: not under test -- always returns 0 */
static inline int bt_le_adv_start(const struct bt_le_adv_param *param,
                                   const struct bt_data *ad, size_t ad_len,
                                   const struct bt_data *sd, size_t sd_len)
{
    (void)param;(void)ad;(void)ad_len;(void)sd;(void)sd_len;
    return 0;
}

/* bt_le_adv_update_data and bt_le_adv_stop are declared here so the
 * compiler sees the prototype; the FFF mock in test_reed_main.c provides
 * the actual definition at link time. */
int bt_le_adv_update_data(const struct bt_data *ad, size_t ad_len,
                           const struct bt_data *sd, size_t sd_len);
int bt_le_adv_stop(void);

#endif /* STUB_BLUETOOTH_H */
