#ifndef STUB_ESP_HTTP_CLIENT_H
#define STUB_ESP_HTTP_CLIENT_H

#include "esp_err.h"
#include <stddef.h>

typedef enum {
    HTTP_METHOD_GET = 0,
    HTTP_METHOD_POST,
} esp_http_client_method_t;

typedef enum {
    HTTP_TRANSPORT_OVER_TCP = 0,
    HTTP_TRANSPORT_OVER_SSL,
} esp_http_client_transport_t;

typedef enum {
    HTTP_EVENT_ERROR = 0,
    HTTP_EVENT_ON_CONNECTED,
    HTTP_EVENT_ON_DATA,
    HTTP_EVENT_ON_FINISH,
} esp_http_client_event_id_t;

typedef void *esp_http_client_handle_t;

typedef struct {
    esp_http_client_event_id_t event_id;
    void   *data;
    int     data_len;
    char   *header_key;
    char   *header_value;
} esp_http_client_event_t;

typedef esp_err_t (*http_event_handle_cb)(esp_http_client_event_t *evt);

typedef struct {
    const char                  *url;
    const char                  *cert_pem;
    void                        (*crt_bundle_attach)(void *);
    http_event_handle_cb         event_handler;
    void                        *user_data;
    int                          timeout_ms;
    esp_http_client_method_t     method;
    esp_http_client_transport_t  transport_type;
} esp_http_client_config_t;

static inline esp_http_client_handle_t esp_http_client_init(const esp_http_client_config_t *c)
{ (void)c; return NULL; }
static inline esp_err_t esp_http_client_set_header(esp_http_client_handle_t c, const char *k, const char *v)
{ (void)c;(void)k;(void)v; return ESP_OK; }
static inline esp_err_t esp_http_client_set_post_field(esp_http_client_handle_t c, const char *d, int l)
{ (void)c;(void)d;(void)l; return ESP_OK; }
static inline esp_err_t esp_http_client_perform(esp_http_client_handle_t c)
{ (void)c; return ESP_OK; }
static inline int esp_http_client_get_status_code(esp_http_client_handle_t c)
{ (void)c; return 200; }
static inline esp_err_t esp_http_client_cleanup(esp_http_client_handle_t c)
{ (void)c; return ESP_OK; }

#endif /* STUB_ESP_HTTP_CLIENT_H */
