#include "cJSON.h"
#include <stdint.h>
#include <stddef.h>
void trinity_log_event(const char *p) { (void)p; }
void trinity_wdt_kick(void) {}
void trinity_wdt_add(void)  {}

/* Controllable PI stub values -- set from test before calling run_pi_controller() */
float g_stub_kp       = 1.0f;
float g_stub_ki       = 0.05f;
float g_stub_kd       = 0.0f;
int   g_stub_setpoint = 25;
int   g_stub_avg_temp = 25;


/* ble_internal stubs */
void stamp_device(int idx) { (void)idx; }
uint16_t ble_get_device_age_s(int idx) { (void)idx; return 0; }

/* wroom bus stubs */
void bus_publish_pir(uint8_t slot, uint32_t count, int batt)
{ (void)slot;(void)count;(void)batt; }

/* ble_reed stubs */
void bus_publish_reed(uint8_t id, uint8_t state, int batt, const uint8_t *mac)
{ (void)id;(void)state;(void)batt;(void)mac; }
void ble_update_room_sensor(int id, const char *p_state)
{ (void)id;(void)p_state; }

/* ble_temp stubs */
void bus_publish_ble_temp(uint8_t slot, int16_t temp, int batt)
{ (void)slot;(void)temp;(void)batt; }

/* ble_internal shared state stubs */
int      g_ble_gattc_if       = 0xFF;
int      g_gatt_registered    = 0;
void    *connect_queue         = NULL;
int      g_connection_in_progress = 0;
uint8_t  lock_mac[6]          = {0};
int      lock_found            = 0;
int      lock_addr_type        = 0;
uint8_t  light_mac[6]         = {0};
int      light_found           = 0;
int      light_addr_type       = 0;
void ble_scheduler_notify_done(void) {}
void bus_publish_lock(uint8_t state, int batt) { (void)state;(void)batt; }
uint64_t bus_publish_light(uint8_t state) { (void)state; return 0; }

/* ---- cJSON behavioral stub state ---- */
int   g_cjson_parse_ok       = 1;
cJSON g_cjson_kp_item        = { .valuedouble = 1.0,  .valueint = 1  };
cJSON g_cjson_ki_item        = { .valuedouble = 0.05, .valueint = 0  };
cJSON g_cjson_kd_item        = { .valuedouble = 0.0,  .valueint = 0  };
cJSON g_cjson_setpoint_item  = { .valuedouble = 25.0, .valueint = 25 };
cJSON g_cjson_light_item     = { .valuedouble = 0.0,  .valueint = 0  };
cJSON g_cjson_avg_temp_item  = { .valuedouble = 25.0, .valueint = 25 };
int   g_cjson_kp_found       = 1;
int   g_cjson_ki_found       = 1;
int   g_cjson_kd_found       = 1;
int   g_cjson_setpoint_found = 1;
int   g_cjson_light_found    = 1;
int   g_cjson_avg_temp_found = 1;

/* rooms array — defined in hello_uart.c in production, stubbed here */
#include "config.h"
ROOM_SENSOR_T rooms[ROOM_COUNT] = {
    { .sensor_id = 0, .room = "stub_room_0", .state = "closed", .location = "stub" },
    { .sensor_id = 1, .room = "stub_room_1", .state = "closed", .location = "stub" },
};

void bus_publish_temp(int avg_temp) { (void)avg_temp; }

float gateway_get_kp(void)       { return g_stub_kp;       }
float gateway_get_ki(void)       { return g_stub_ki;       }
float gateway_get_kd(void)       { return g_stub_kd;       }
int   gateway_get_setpoint(void) { return g_stub_setpoint; }
int   uart_get_avg_temp(void){ return g_stub_avg_temp; }

/* Controllable tick source for slot table time-based tests */
uint32_t g_stub_tick_ms = 0;
