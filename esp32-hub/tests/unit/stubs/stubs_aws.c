#include <stdint.h>
#include <stddef.h>
#include "cJSON.h"
#include "config.h"
#include <stdbool.h>
#include <stdint.h>

void trinity_log_event(const char *p) { (void)p; }
void trinity_wdt_kick(void) {}
void trinity_wdt_add(void)  {}

/* wroom bus stubs */
void bus_publish_temp(int t)  { (void)t; }
void bus_publish_pir(uint8_t s, uint32_t c, int b) { (void)s;(void)c;(void)b; }
void bus_publish_reed(uint8_t id, uint8_t st, int b, const uint8_t *m) { (void)id;(void)st;(void)b;(void)m; }
void bus_publish_lock(uint8_t s, int b) { (void)s;(void)b; }
void bus_publish_light(uint8_t s) { (void)s; }
void bus_publish_motor(uint8_t o, int b) { (void)o;(void)b; }
void bus_publish_ble_temp(uint8_t s, int16_t t, int b) { (void)s;(void)t;(void)b; }
void ble_update_room_sensor(int id, const char *p) { (void)id;(void)p; }
uint8_t ble_get_light_state(void) { return 0; }
uint8_t ble_get_lock_state(void)  { return 0; }
int     ble_get_lock_batt(void)   { return -1; }
int     ble_get_pir_occupied(int s) { (void)s; return 0; }
int     ble_get_temp_count(void)  { return 0; }
bool    ble_get_temp_slot_info(int s, int16_t *t, int *b, uint8_t *mac) { (void)s;(void)t;(void)b;(void)mac; return false; }

/* rooms array */
ROOM_SENSOR_T rooms[ROOM_COUNT] = {
    { .sensor_id = 0, .room = "stub_room_0", .state = "closed", .location = "stub" },
    { .sensor_id = 1, .room = "stub_room_1", .state = "closed", .location = "stub" },
};

/* cJSON behavioral stub state */
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
void ble_send_light_command(uint8_t s) { (void)s; }

/* Controllable tick source */
uint32_t g_stub_tick_ms = 0;
