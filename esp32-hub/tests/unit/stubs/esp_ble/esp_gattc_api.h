#ifndef STUB_ESP_GATTC_API_H
#define STUB_ESP_GATTC_API_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define ESP_GATT_IF_NONE                 0xFF
#define ESP_GATT_OK                      0
#define ESP_UUID_LEN_16                  2
#define ESP_GATT_DB_CHARACTERISTIC       0
#define ESP_GATT_AUTH_REQ_NONE           0
#define ESP_GATT_WRITE_TYPE_RSP          0
#define ESP_GATT_UUID_CHAR_CLIENT_CONFIG 0x2902

typedef int     esp_gatt_if_t;
typedef int     esp_gattc_cb_event_t;
typedef uint8_t esp_gatt_status_t;
typedef uint8_t esp_gatt_write_type_t;
typedef uint8_t esp_gatt_auth_req_t;

typedef union {
    uint16_t uuid16;
} esp_bt_uuid_val_t;

typedef struct {
    uint16_t          len;
    esp_bt_uuid_val_t uuid;
} esp_bt_uuid_t;

/* char_handle matches the field name used in ble_light.c */
typedef struct {
    esp_bt_uuid_t uuid;
    uint16_t      char_handle;
} esp_gattc_char_elem_t;

typedef struct {
    uint16_t handle;
} esp_gattc_descr_elem_t;

typedef struct {
    esp_bt_uuid_t uuid;
    uint16_t      start_handle;
    uint16_t      end_handle;
} esp_gattc_srvc_elem_t;

typedef union {
    struct {
        uint16_t conn_id;
        uint8_t  remote_bda[6];
    } connect;

    struct {
        uint16_t conn_id;
        int      status;
    } open;

    struct {
        uint16_t conn_id;
        uint16_t mtu;
    } cfg_mtu;

    /* search_res: srvc_id.uuid needs .len and .uuid.uuid16;
       start_handle at the top level of the struct (line 385 of ble_light.c) */
    struct {
        uint16_t conn_id;
        uint16_t start_handle;
        uint16_t end_handle;
        struct {
            esp_bt_uuid_t uuid;   /* .len and .uuid.uuid16 live here */
        } srvc_id;
    } search_res;

    struct {
        uint16_t conn_id;
    } search_cmpl;

    struct {
        uint16_t  conn_id;
        int       status;
        uint16_t  handle;
        uint16_t  value_len;
        uint8_t  *value;
    } read;

    struct {
        uint16_t conn_id;
        int      status;
        uint16_t handle;
    } write;

    struct {
        uint16_t conn_id;
        uint8_t  reason;
    } disconnect;

    struct {
        uint16_t  conn_id;
        uint16_t  handle;
        uint16_t  value_len;
        uint8_t  *value;
        int       is_notify;
    } notify;
} esp_ble_gattc_cb_param_t;

#define ESP_GATTC_CONNECT_EVT      0
#define ESP_GATTC_OPEN_EVT         1
#define ESP_GATTC_CFG_MTU_EVT      2
#define ESP_GATTC_SEARCH_RES_EVT   3
#define ESP_GATTC_SEARCH_CMPL_EVT  4
#define ESP_GATTC_READ_CHAR_EVT    5
#define ESP_GATTC_NOTIFY_EVT       6
#define ESP_GATTC_WRITE_CHAR_EVT   7
#define ESP_GATTC_WRITE_DESCR_EVT  8
#define ESP_GATTC_DISCONNECT_EVT   9

static inline esp_err_t esp_ble_gattc_open(int i, uint8_t *m, int t, int d)
{ (void)i;(void)m;(void)t;(void)d; return ESP_OK; }
static inline esp_err_t esp_ble_gattc_close(int i, uint16_t c)
{ (void)i;(void)c; return ESP_OK; }
static inline esp_err_t esp_ble_gattc_send_mtu_req(int i, uint16_t c)
{ (void)i;(void)c; return ESP_OK; }
static inline esp_err_t esp_ble_gattc_search_service(int i, uint16_t c, void *u)
{ (void)i;(void)c;(void)u; return ESP_OK; }
static inline esp_err_t esp_ble_gattc_get_attr_count(int i, uint16_t c, int t,
    uint16_t s, uint16_t e, uint16_t h, uint16_t *cnt)
{ (void)i;(void)c;(void)t;(void)s;(void)e;(void)h; *cnt=0; return ESP_OK; }
static inline esp_err_t esp_ble_gattc_get_all_char(int i, uint16_t c,
    uint16_t s, uint16_t e, void *p, uint16_t *cnt, uint16_t o)
{ (void)i;(void)c;(void)s;(void)e;(void)p;(void)cnt;(void)o; return ESP_OK; }
static inline esp_err_t esp_ble_gattc_read_char(int i, uint16_t c, uint16_t h, int a)
{ (void)i;(void)c;(void)h;(void)a; return ESP_OK; }
static inline esp_err_t esp_ble_gattc_write_char(int i, uint16_t c, uint16_t h,
    uint16_t l, uint8_t *v, int t, int a)
{ (void)i;(void)c;(void)h;(void)l;(void)v;(void)t;(void)a; return ESP_OK; }
static inline esp_err_t esp_ble_gattc_write_char_descr(int i, uint16_t c, uint16_t h,
    uint16_t l, uint8_t *v, int t, int a)
{ (void)i;(void)c;(void)h;(void)l;(void)v;(void)t;(void)a; return ESP_OK; }
static inline esp_err_t esp_ble_gattc_get_descr_by_char_handle(int i, uint16_t c,
    uint16_t h, esp_bt_uuid_t u, esp_gattc_descr_elem_t *d, uint16_t *cnt)
{ (void)i;(void)c;(void)h;(void)u;(void)d; *cnt=0; return ESP_OK; }
static inline const char *esp_err_to_name(int e) { (void)e; return "stub"; }

#endif /* STUB_ESP_GATTC_API_H */
