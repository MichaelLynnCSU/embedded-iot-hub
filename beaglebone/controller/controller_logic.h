#ifndef CONTROLLER_LOGIC_H
#define CONTROLLER_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

/* ---- Sizing constants ---- */
#define MAX_ROOMS          10
#define MAX_REEDS          6
#define DEVICE_COUNT       6
#define HISTORY_BUF_SIZE   100
#define ALERT_BUF_SIZE     10
#define ROOM_BUF_SIZE      10
#define ALERT_MSG_SIZE     64
#define HB_TIMEOUT_SEC     10
#define HB_POLL_SEC        1
#define UART_LINE_LEN      64
#define ROOM_QUERY_LIMIT   10

/* ---- Alert severity levels ---- */
#define ALERT_SEVERITY_LOW   1
#define ALERT_SEVERITY_MED   2
#define ALERT_SEVERITY_HIGH  3

/* ---- Command enum values ---- */
#define CMD_GET_LATEST        1
#define CMD_GET_DEVICE_STATUS 24
#define CMD_GET_ROOM_STATUS   21

/* ---- DEV_ID_E ---- */
typedef enum
{
    DEV_PIR   = 0,
    DEV_LIGHT = 1,
    DEV_LOCK  = 2,
    DEV_MOTOR = 3,
    DEV_COUNT = 4,
} DEV_ID_E;

/* ---- Pure logic: heartbeat online check ---- */
static inline uint8_t logic_hb_is_online(time_t last_seen, time_t now)
{
    if (0 == last_seen) { return 0; }
    return (uint8_t)((now - last_seen) < HB_TIMEOUT_SEC);
}

/* ---- Pure logic: heartbeat state changed ---- */
static inline bool logic_hb_state_changed(uint8_t was_online,
                                           uint8_t is_online)
{
    return was_online != is_online;
}

/* ---- Pure logic: device index valid ---- */
static inline bool logic_dev_idx_valid(int idx)
{
    return (idx >= 0) && (idx < (int)DEV_COUNT);
}

/* ---- Pure logic: reed slot valid ---- */
static inline bool logic_reed_slot_valid(int slot)
{
    return (slot >= 0) && (slot < MAX_REEDS);
}

/* ---- Pure logic: alert severity label ---- */
static inline const char *logic_severity_label(int severity)
{
    switch (severity)
    {
        case ALERT_SEVERITY_LOW:  return "LOW";
        case ALERT_SEVERITY_MED:  return "MEDIUM";
        case ALERT_SEVERITY_HIGH: return "HIGH";
        default:                  return "UNKNOWN";
    }
}

/* ---- Pure logic: history ring buffer index ---- */
static inline int logic_history_idx(int count)
{
    return count % HISTORY_BUF_SIZE;
}

/* ---- Pure logic: batt sentinel ---- */
static inline bool logic_batt_valid(int batt)
{
    return batt >= 0;
}

/* ---- Pure logic: command is known ---- */
static inline bool logic_cmd_known(int cmd)
{
    return (cmd >= CMD_GET_LATEST) && (cmd <= CMD_GET_DEVICE_STATUS);
}

/* ---- Pure logic: should reed slot be updated from incoming frame ---- */
/* Phantom widget fix (2026-04-21):
 * Only overwrite latest slot data when incoming frame has active=1.
 * Preserves previous state when a slot is missing from one JSON frame. */
static inline bool logic_reed_slot_should_update(uint8_t incoming_active)
{
    return incoming_active != 0;
}

#endif /* CONTROLLER_LOGIC_H */
