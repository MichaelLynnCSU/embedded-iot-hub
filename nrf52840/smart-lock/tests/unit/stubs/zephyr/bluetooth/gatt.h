#ifndef STUB_GATT_H
#define STUB_GATT_H

/*
 * Stub: zephyr/bluetooth/gatt.h
 * ble_gatt.c uses BT_GATT_SERVICE_DEFINE, BT_GATT_PRIMARY_SERVICE,
 * BT_GATT_CHARACTERISTIC, BT_GATT_CCC, bt_gatt_notify(),
 * bt_gatt_attr_read(), BT_ATT_ERR_*, BT_GATT_ERR(), ssize_t.
 *
 * Under test we only exercise lock_state_* and mv_to_soc() logic.
 * ble_gatt.c is compiled in so its symbols resolve, but GATT functions
 * are stubbed to no-ops so no BLE stack is needed.
 */

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>  /* ssize_t */

struct bt_conn;

struct bt_gatt_attr {
    const void *uuid;
    void       *user_data;
    uint16_t    handle;
};

struct bt_gatt_service_static {
    const struct bt_gatt_attr *attrs;
    size_t                     attr_count;
};

/* UUIDs -- opaque pointers suffice */
#define BT_UUID_DECLARE_16(val)   ((const void *)(uintptr_t)(val))

/* Attribute permission / property flags */
#define BT_GATT_PERM_READ              0x01
#define BT_GATT_PERM_WRITE             0x02
#define BT_GATT_CHRC_READ              0x02
#define BT_GATT_CHRC_WRITE             0x08
#define BT_GATT_CHRC_WRITE_WITHOUT_RESP 0x04
#define BT_GATT_CHRC_NOTIFY            0x10

/* ATT error codes */
#define BT_ATT_ERR_INVALID_ATTRIBUTE_LEN  0x0D
#define BT_ATT_ERR_VALUE_NOT_ALLOWED      0x13
#define BT_GATT_ERR(err)                  (-(err))

/* Service/characteristic definition macros -- emit a dummy attr entry each */
#define BT_GATT_PRIMARY_SERVICE(uuid)        { (uuid), NULL, 0 }
#define BT_GATT_CHARACTERISTIC(uuid, props, perms, read, write, data) \
                                             { (uuid), (data), 0 }
#define BT_GATT_CCC(changed, perms)          { NULL,   NULL,  0 }

/* BT_GATT_SERVICE_DEFINE -- declare a static service with an attrs array.
 * We emit a real array so &svc.attrs[N] resolves at compile time. */
#define BT_GATT_SERVICE_DEFINE(name, ...)                              \
    static const struct bt_gatt_attr _##name##_attrs[] = { __VA_ARGS__ }; \
    static const struct bt_gatt_service_static name = {                \
        .attrs      = _##name##_attrs,                                 \
        .attr_count = sizeof(_##name##_attrs) /                        \
                      sizeof(_##name##_attrs[0]),                      \
    }

/* Runtime GATT API -- all no-ops under test */
static inline int bt_gatt_notify(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *data, uint16_t len)
{ (void)conn;(void)attr;(void)data;(void)len; return 0; }

static inline ssize_t bt_gatt_attr_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t buf_len,
                                         uint16_t offset,
                                         const void *value, uint16_t value_len)
{ (void)conn;(void)attr;(void)buf;(void)buf_len;(void)offset;
  (void)value;(void)value_len; return (ssize_t)value_len; }

/* bt_conn callbacks struct -- fields not needed, just needs to exist */
struct bt_conn_cb {
    void (*connected)(struct bt_conn *, uint8_t);
    void (*disconnected)(struct bt_conn *, uint8_t);
};

static inline int bt_conn_disconnect(struct bt_conn *c, uint8_t r)
{ (void)c;(void)r; return 0; }
static inline struct bt_conn *bt_conn_ref(struct bt_conn *c) { return c; }
static inline void bt_conn_unref(struct bt_conn *c) { (void)c; }

#endif /* STUB_GATT_H */
