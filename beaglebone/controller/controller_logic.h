/******************************************************************************
 * \file controller_logic.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Pure logic and state machines for BeagleBone data controller.
 *
 * \details All functions are pure — no I/O, no globals, no side effects.
 *          Safe to unit test without hardware or OS dependencies.
 *
 *          State machines:
 *          - LOCK_STATE_E:  models lock/unlock/moving/error transitions
 *          - MOTOR_STATE_E: models motor online/offline transitions
 *
 *          Both machines use the same pattern:
 *          - logic_*_transition() validates and returns the new state
 *          - logic_*_event_str() returns the DB event string for a transition
 *          - Caller owns all side effects (DB writes, shm updates, logging)
 ******************************************************************************/

#ifndef CONTROLLER_LOGIC_H
#define CONTROLLER_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

/* controller_logic.h is included after controller_internal.h —
 * all sizing constants, DEV_ID_E, and command values are defined there. */

/**
 * \brief Lock state machine states.
 *
 * \details Valid transitions:
 *          LOCKED    → UNLOCKING  (unlock command received)
 *          UNLOCKING → UNLOCKED   (motor confirm or timeout)
 *          UNLOCKED  → LOCKING    (lock command received)
 *          LOCKING   → LOCKED     (motor confirm or timeout)
 *          ANY       → ERROR      (motor fault or comms loss)
 *          ERROR     → LOCKED     (recovery — assume locked for safety)
 *
 *          UNLOCKING and LOCKING reject all commands — motor is moving.
 */
typedef enum
{
    LOCK_STATE_LOCKED    = 0,  /**< locked — idle, accepts unlock command */
    LOCK_STATE_UNLOCKING = 1,  /**< transitioning locked → unlocked */
    LOCK_STATE_UNLOCKED  = 2,  /**< unlocked — idle, accepts lock command */
    LOCK_STATE_LOCKING   = 3,  /**< transitioning unlocked → locked */
    LOCK_STATE_ERROR     = 4,  /**< fault — motor or comms error */
} LOCK_STATE_E;

/**
 * \brief Motor controller state machine states.
 *
 * \details Valid transitions:
 *          OFFLINE → ONLINE   (heartbeat received)
 *          ONLINE  → OFFLINE  (heartbeat timeout)
 *          ANY     → FAULT    (unexpected motor behaviour)
 */
typedef enum
{
    MOTOR_STATE_OFFLINE = 0,  /**< motor controller not responding */
    MOTOR_STATE_ONLINE  = 1,  /**< motor controller heartbeat active */
    MOTOR_STATE_FAULT   = 2,  /**< motor controller fault detected */
} MOTOR_STATE_E;

/*************************** LOCK STATE MACHINE *******************************/

/******************************************************************************
 * \brief Validate and return new lock state for a requested transition.
 *
 * \param current - Current lock state.
 * \param cmd     - Requested lock value (0=lock, 1=unlock).
 *
 * \return LOCK_STATE_E - New state if transition is valid, current if rejected.
 *
 * \details Rejects all commands while motor is moving (UNLOCKING/LOCKING).
 *          Rejects unlock when already unlocked, lock when already locked.
 *          ERROR state only accepts recovery to LOCKED.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline LOCK_STATE_E logic_lock_transition(LOCK_STATE_E current,
                                                  int          cmd)
{
    switch (current)
    {
        case LOCK_STATE_LOCKED:
            return (1 == cmd) ? LOCK_STATE_UNLOCKING : current;

        case LOCK_STATE_UNLOCKED:
            return (0 == cmd) ? LOCK_STATE_LOCKING : current;

        case LOCK_STATE_ERROR:
            /* safety — only allow recovery to locked */
            return (0 == cmd) ? LOCK_STATE_LOCKED : current;

        case LOCK_STATE_UNLOCKING:
        case LOCK_STATE_LOCKING:
            /* motor moving — reject all commands */
            return current;

        default:
            return LOCK_STATE_ERROR;
    }
}

/******************************************************************************
 * \brief Advance lock state after motor confirms movement complete.
 *
 * \param current - Current lock state (must be UNLOCKING or LOCKING).
 *
 * \return LOCK_STATE_E - Settled state after motor completes.
 *
 * \details Called when motor heartbeat confirms position reached or
 *          movement timeout expires. UNLOCKING → UNLOCKED, LOCKING → LOCKED.
 *          All other states returned unchanged.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline LOCK_STATE_E logic_lock_settle(LOCK_STATE_E current)
{
    switch (current)
    {
        case LOCK_STATE_UNLOCKING: return LOCK_STATE_UNLOCKED;
        case LOCK_STATE_LOCKING:   return LOCK_STATE_LOCKED;
        default:                   return current;
    }
}

/******************************************************************************
 * \brief Return DB event string for a lock state transition.
 *
 * \param from - Previous lock state.
 * \param to   - New lock state.
 *
 * \return const char* - Event string for device_events table, or NULL if
 *                       no event should be logged for this transition.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline const char *logic_lock_event_str(LOCK_STATE_E from,
                                                LOCK_STATE_E to)
{
    (void)from;

    switch (to)
    {
        case LOCK_STATE_LOCKED:    return "locked";
        case LOCK_STATE_UNLOCKED:  return "unlocked";
        case LOCK_STATE_UNLOCKING: return "unlocking";
        case LOCK_STATE_LOCKING:   return "locking";
        case LOCK_STATE_ERROR:     return "error";
        default:                   return NULL;
    }
}

/******************************************************************************
 * \brief Return human-readable label for a lock state.
 *
 * \param state - Lock state value.
 *
 * \return const char* - State label string.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline const char *logic_lock_state_label(LOCK_STATE_E state)
{
    switch (state)
    {
        case LOCK_STATE_LOCKED:    return "LOCKED";
        case LOCK_STATE_UNLOCKING: return "UNLOCKING";
        case LOCK_STATE_UNLOCKED:  return "UNLOCKED";
        case LOCK_STATE_LOCKING:   return "LOCKING";
        case LOCK_STATE_ERROR:     return "ERROR";
        default:                   return "UNKNOWN";
    }
}

/******************************************************************************
 * \brief Return true if lock is busy (motor moving) — commands rejected.
 *
 * \param state - Current lock state.
 *
 * \return bool - true if commands should be rejected.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline bool logic_lock_is_busy(LOCK_STATE_E state)
{
    return (LOCK_STATE_UNLOCKING == state) || (LOCK_STATE_LOCKING == state);
}

/************************** MOTOR STATE MACHINE *******************************/

/******************************************************************************
 * \brief Validate and return new motor state for a heartbeat update.
 *
 * \param current   - Current motor state.
 * \param hb_online - 1 if heartbeat is active, 0 if timed out.
 *
 * \return MOTOR_STATE_E - New motor state.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline MOTOR_STATE_E logic_motor_transition(MOTOR_STATE_E current,
                                                    int           hb_online)
{
    switch (current)
    {
        case MOTOR_STATE_OFFLINE:
            return (1 == hb_online) ? MOTOR_STATE_ONLINE : current;

        case MOTOR_STATE_ONLINE:
            return (0 == hb_online) ? MOTOR_STATE_OFFLINE : current;

        case MOTOR_STATE_FAULT:
            /* fault cleared only by explicit online heartbeat */
            return (1 == hb_online) ? MOTOR_STATE_ONLINE : current;

        default:
            return MOTOR_STATE_FAULT;
    }
}

/******************************************************************************
 * \brief Return DB event string for a motor state transition.
 *
 * \param from - Previous motor state.
 * \param to   - New motor state.
 *
 * \return const char* - Event string for device_events table, or NULL if
 *                       no event should be logged.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline const char *logic_motor_event_str(MOTOR_STATE_E from,
                                                 MOTOR_STATE_E to)
{
    (void)from;

    switch (to)
    {
        case MOTOR_STATE_ONLINE:  return "ONLINE";
        case MOTOR_STATE_OFFLINE: return "OFFLINE";
        case MOTOR_STATE_FAULT:   return "FAULT";
        default:                  return NULL;
    }
}

/******************************************************************************
 * \brief Return human-readable label for a motor state.
 *
 * \param state - Motor state value.
 *
 * \return const char* - State label string.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline const char *logic_motor_state_label(MOTOR_STATE_E state)
{
    switch (state)
    {
        case MOTOR_STATE_OFFLINE: return "OFFLINE";
        case MOTOR_STATE_ONLINE:  return "ONLINE";
        case MOTOR_STATE_FAULT:   return "FAULT";
        default:                  return "UNKNOWN";
    }
}

/************************* HEARTBEAT PURE LOGIC *******************************/

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
