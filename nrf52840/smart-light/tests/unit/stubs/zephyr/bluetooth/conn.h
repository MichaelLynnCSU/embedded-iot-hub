#ifndef STUB_CONN_H
#define STUB_CONN_H
/* Stub: zephyr/bluetooth/conn.h -- bt_conn opaque type already in gatt.h */
#include <zephyr/bluetooth/gatt.h>

struct bt_le_conn_param {
    uint16_t interval_min;
    uint16_t interval_max;
    uint16_t latency;
    uint16_t timeout;
};

static inline int bt_conn_le_param_update(struct bt_conn *c,
                                           const struct bt_le_conn_param *p)
{ (void)c;(void)p; return 0; }

static inline const bt_addr_le_t *bt_conn_get_dst(const struct bt_conn *c)
{ (void)c; return NULL; }

static inline void bt_addr_le_to_str(const bt_addr_le_t *addr,
                                      char *str, size_t len)
{ (void)addr; if (str && len) str[0] = '\0'; }

#endif /* STUB_CONN_H */
