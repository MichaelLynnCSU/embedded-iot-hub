#ifndef HUB_LOGIC_H
#define HUB_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ---- Timing constants ---- */
#define STATS_INTERVAL_MS       60000u
#define WIFI_POLL_INTERVAL_MS   2000u
#define TCP_SEND_INTERVAL_MS    1000u
#define AWS_SEND_INTERVAL_MS    300000u
#define DRAIN_INTERVAL_MS       2000u
#define RECONNECT_DELAY_MS      2000u
#define BB_CONNECT_TIMEOUT_MS   2000u
#define C3_CONNECT_TIMEOUT_MS   10000u
#define BLOCK_COUNT_MAX         5

/* ---- Reed slot constants ---- */
#define MAX_REEDS               6
#define REED_OFFLINE_MS         (150  * 1000)
#define REED_REMOVE_MS          (3600 * 1000)
#define REED_OFFLINE_S          150
#define COOLDOWN_MS             (10   * 1000)

/* ---- MFG data byte indices ---- */
#define MFG_PIR_MIN_LEN         6
#define MFG_PIR_BATT_IDX        6
#define MFG_REED_STATE_IDX      1
#define MFG_REED_BATT_IDX       2
#define MFG_LIGHT_STATE_IDX     2
#define MFG_LIGHT_MIN_LEN       2
#define MFG_LOCK_STATE_IDX      1
#define MFG_LOCK_BATT_IDX       2
#define MFG_LOCK_MIN_LEN        3

/* ---- PIR count byte indices ---- */
#define PIR_COUNT_BYTE0         2
#define PIR_COUNT_BYTE1         3
#define PIR_COUNT_BYTE2         4
#define PIR_COUNT_BYTE3         5

/* ---- Default values ---- */
#define DEFAULT_AVG_TEMP        25
#define DEFAULT_AWS_LOW         20
#define DEFAULT_AWS_HIGH        35
#define DEFAULT_AWS_MOTOR       0
#define DEFAULT_MOTION_COUNT    0

/* ---- WiFi backoff ---- */
#define WIFI_BACKOFF_TABLE_SIZE 5

/* ---- Reed name prefix ---- */
#define REED_NAME_PREFIX        "ReedSensor"
#define REED_NAME_PREFIX_LEN    10

/* ---- TCP state machine ---- */
#define TCP_STATE_DISCONNECTED  0
#define TCP_STATE_CONNECTING    1
#define TCP_STATE_CONNECTED     2
#define SOCK_INVALID            -1

/* ---- Pure logic: PIR count unpacking ---- */
static inline uint32_t hub_unpack_pir_count(const uint8_t *p_mfg)
{
    return ((uint32_t)p_mfg[PIR_COUNT_BYTE0] << 24) |
           ((uint32_t)p_mfg[PIR_COUNT_BYTE1] << 16) |
           ((uint32_t)p_mfg[PIR_COUNT_BYTE2] <<  8) |
            (uint32_t)p_mfg[PIR_COUNT_BYTE3];
}

/* ---- Pure logic: reed offline check ---- */
static inline bool hub_reed_is_offline(uint32_t age_ms)
{
    return age_ms > REED_OFFLINE_MS;
}

static inline bool hub_reed_should_remove(uint32_t age_ms)
{
    return age_ms > REED_REMOVE_MS;
}

/* ---- Pure logic: reed offline seconds check ---- */
static inline uint8_t hub_reed_offline_flag(uint16_t age_s)
{
    return (age_s > REED_OFFLINE_S) ? 1u : 0u;
}

/* ---- Pure logic: reed name prefix match ---- */
static inline bool hub_is_reed_name(const char *name)
{
    return strncmp(name, REED_NAME_PREFIX, REED_NAME_PREFIX_LEN) == 0;
}

/* ---- Pure logic: WiFi backoff table ---- */
static const int hub_wifi_backoff_sec[WIFI_BACKOFF_TABLE_SIZE] =
{
    2, 5, 10, 30, 60
};

#endif /* HUB_LOGIC_H */
