#ifndef STUB_CJSON_H
#define STUB_CJSON_H

#include <stdint.h>
#include <stddef.h>

/* Minimal cJSON type stub */
typedef struct cJSON {
    struct cJSON *next;
    struct cJSON *prev;
    struct cJSON *child;
    int    type;
    char  *valuestring;
    int    valueint;
    double valuedouble;
    char  *string;
} cJSON;

/* ---- Controllable stub state ---- */
/* Set these from tests to drive parse behavior */
extern int    g_cjson_parse_ok;       /* 1 = Parse returns object, 0 = NULL */
extern cJSON  g_cjson_kp_item;
extern cJSON  g_cjson_ki_item;
extern cJSON  g_cjson_kd_item;
extern cJSON  g_cjson_setpoint_item;
extern cJSON  g_cjson_light_item;
extern cJSON  g_cjson_avg_temp_item;
extern int    g_cjson_kp_found;
extern int    g_cjson_ki_found;
extern int    g_cjson_kd_found;
extern int    g_cjson_setpoint_found;
extern int    g_cjson_light_found;
extern int    g_cjson_avg_temp_found;

/* ---- Behavioral stubs ---- */
static inline cJSON *cJSON_Parse(const char *s)
{ (void)s; extern int g_cjson_parse_ok; return g_cjson_parse_ok ? (cJSON *)1 : NULL; }

static inline cJSON *cJSON_GetObjectItem(const cJSON *obj, const char *key);

static inline void cJSON_Delete(cJSON *p) { (void)p; }
static inline void cJSON_free(void *p)    { (void)p; }

/* ---- Compile-only serialization stubs ---- */
static inline cJSON *cJSON_CreateObject(void) { return NULL; }
static inline cJSON *cJSON_CreateArray(void)  { return NULL; }
static inline cJSON *cJSON_AddNumberToObject(cJSON *o, const char *n, double v)
{ (void)o;(void)n;(void)v; return NULL; }
static inline cJSON *cJSON_AddStringToObject(cJSON *o, const char *n, const char *s)
{ (void)o;(void)n;(void)s; return NULL; }
static inline void cJSON_AddItemToObject(cJSON *o, const char *n, cJSON *i)
{ (void)o;(void)n;(void)i; }
static inline void cJSON_AddItemToArray(cJSON *a, cJSON *i)
{ (void)a;(void)i; }
static inline char *cJSON_PrintUnformatted(const cJSON *o) { (void)o; return NULL; }

/* GetObjectItem needs string comparison — implement after includes */
#include <string.h>
static inline cJSON *cJSON_GetObjectItem(const cJSON *obj, const char *key)
{
    (void)obj;
    extern cJSON g_cjson_kp_item;       extern int g_cjson_kp_found;
    extern cJSON g_cjson_ki_item;       extern int g_cjson_ki_found;
    extern cJSON g_cjson_kd_item;       extern int g_cjson_kd_found;
    extern cJSON g_cjson_setpoint_item; extern int g_cjson_setpoint_found;
    extern cJSON g_cjson_light_item;    extern int g_cjson_light_found;
    extern cJSON g_cjson_avg_temp_item; extern int g_cjson_avg_temp_found;
    if (strcmp(key, "kp")       == 0) return g_cjson_kp_found       ? &g_cjson_kp_item       : NULL;
    if (strcmp(key, "ki")       == 0) return g_cjson_ki_found       ? &g_cjson_ki_item       : NULL;
    if (strcmp(key, "kd")       == 0) return g_cjson_kd_found       ? &g_cjson_kd_item       : NULL;
    if (strcmp(key, "setpoint") == 0) return g_cjson_setpoint_found ? &g_cjson_setpoint_item : NULL;
    if (strcmp(key, "light")    == 0) return g_cjson_light_found    ? &g_cjson_light_item    : NULL;
    if (strcmp(key, "avg_temp") == 0) return g_cjson_avg_temp_found ? &g_cjson_avg_temp_item : NULL;
    return NULL;
}

#endif /* STUB_CJSON_H */
