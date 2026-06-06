#ifndef STUB_BLUETOOTH_H
#define STUB_BLUETOOTH_H

/*
 * Stub: zephyr/bluetooth/bluetooth.h
 * ble_gatt.c uses bt_enable(), bt_le_adv_start(), bt_le_adv_update_data(),
 * bt_le_adv_stop(), BT_CONN_CB_DEFINE, BT_GATT_SERVICE_DEFINE, and the
 * BT_DATA / BT_LE_ADV_OPT_* macros.
 *
 * bt_le_adv_update_data() is FFF-mocked in the test file.
 * The rest are stubbed here so ble_gatt.c compiles cleanly.
 */

#include <stdint.h>
#include <stddef.h>

/* bt_data: advertising payload element */
struct bt_data {
    uint8_t        type;
    uint8_t        data_len;
    const uint8_t *data;
};

/* bt_le_adv_param */
struct bt_le_adv_param {
    uint8_t  id;
    uint32_t options;
    uint16_t interval_min;
    uint16_t interval_max;
};

/* bt_addr_le_t */
typedef struct {
    uint8_t type;
    uint8_t a[6];
} bt_addr_le_t;

/* Manifest constants */
#define BT_ID_DEFAULT              0
#define BT_LE_ADV_OPT_CONN         (1U << 0)
#define BT_LE_ADV_OPT_USE_IDENTITY (1U << 1)
#define BT_LE_AD_GENERAL           0x02
#define BT_LE_AD_NO_BREDR          0x04
#define BT_DATA_FLAGS              0x01
#define BT_DATA_NAME_COMPLETE      0x09
#define BT_DATA_UUID16_ALL         0x03
#define BT_DATA_MANUFACTURER_DATA  0xFF
#define BT_GAP_ADV_SLOW_INT_MIN    0x0640
#define BT_GAP_ADV_SLOW_INT_MAX    0x0780
#define BT_ADDR_LE_STR_LEN         30
#define BT_HCI_ERR_REMOTE_USER_TERM_CONN 0x13

/* Convenience constructors */
#define BT_DATA_BYTES(type, ...) \
    { (type), sizeof((uint8_t[]){__VA_ARGS__}), (uint8_t[]){__VA_ARGS__} }
#define BT_DATA(type, data_ptr, data_len_val) \
    { (type), (data_len_val), (const uint8_t *)(data_ptr) }

#ifndef CONFIG_BT_DEVICE_NAME
#define CONFIG_BT_DEVICE_NAME "LightTest"
#endif

/* BT_CONN_CB_DEFINE -- swallow the callback registration */
#define BT_CONN_CB_DEFINE(name) \
    static const struct bt_conn_cb name __attribute__((unused))

static inline int bt_enable(void *cb) { (void)cb; return 0; }

static inline int bt_le_adv_start(const struct bt_le_adv_param *p,
                                   const struct bt_data *ad, size_t al,
                                   const struct bt_data *sd, size_t sl)
{ (void)p;(void)ad;(void)al;(void)sd;(void)sl; return 0; }

/* FFF-mocked in test file -- declare only */
int bt_le_adv_update_data(const struct bt_data *ad, size_t ad_len,
                           const struct bt_data *sd, size_t sd_len);
int bt_le_adv_stop(void);

/* BT_LE_ADV_CONN_FAST_1 -- used by ble_gatt.c adv start/restart */
static const struct bt_le_adv_param _bt_le_adv_conn_fast_1 = {
    .id           = BT_ID_DEFAULT,
    .options      = BT_LE_ADV_OPT_CONN,
    .interval_min = 0x0030,
    .interval_max = 0x0060,
};
#define BT_LE_ADV_CONN_FAST_1 (&_bt_le_adv_conn_fast_1)

#endif /* STUB_BLUETOOTH_H */
