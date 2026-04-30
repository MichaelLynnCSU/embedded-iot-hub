/******************************************************************************
 * \file    lock_state.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Lock state machine for nRF52840 smart lock node.
 *
 * \details Pure logic — no Zephyr deps, no I/O, no globals, no side effects.
 *          Safe to unit test without hardware or OS dependencies.
 *          Mirrors LOCK_STATE_E and logic_lock_* in BeagleBone
 *          controller_logic.h — both ends model the same transitions.
 *
 *          Valid transitions:
 *          LOCKED    -> UNLOCKING   phone sends unlock (cmd=0)
 *          UNLOCKING -> UNLOCKED    motor_off_timer fires (settle)
 *          UNLOCKED  -> LOCKING     phone sends lock   (cmd=1)
 *          LOCKING   -> LOCKED      motor_off_timer fires (settle)
 *          ANY       -> ERROR       fault detected
 *          ERROR     -> LOCKED      recovery — assume locked for safety
 *
 *          UNLOCKING and LOCKING reject all commands — motor is moving.
 *
 *          Caller owns all side effects:
 *          - BLE GATT notify on transition
 *          - Trinity log event on transition
 *          - motor_drive() call
 *          - BLE adv update
 ******************************************************************************/

#ifndef LOCK_STATE_H
#define LOCK_STATE_H

#include <stdint.h>
#include <stdbool.h>

/******************************* ENUMERATIONS *********************************/

/**
 * \brief Lock state machine states.
 *
 * \details Matches LOCK_STATE_E in BeagleBone controller_logic.h.
 *          Values are stable — stored in settings and sent over BLE adv.
 */
typedef enum
{
    LOCK_STATE_LOCKED    = 0,  /**< locked — idle, accepts unlock command */
    LOCK_STATE_UNLOCKING = 1,  /**< transitioning locked -> unlocked */
    LOCK_STATE_UNLOCKED  = 2,  /**< unlocked — idle, accepts lock command */
    LOCK_STATE_LOCKING   = 3,  /**< transitioning unlocked -> locked */
    LOCK_STATE_ERROR     = 4,  /**< fault — motor or comms error */
} LOCK_STATE_E;

/*************************** STATE MACHINE LOGIC ******************************/

/******************************************************************************
 * \brief Validate and return new lock state for a phone command.
 *
 * \param current - Current lock state.
 * \param cmd     - Requested lock value (1=lock, 0=unlock).
 *
 * \return LOCK_STATE_E - New state if transition valid, current if rejected.
 *
 * \details Rejects all commands while motor is moving (UNLOCKING/LOCKING).
 *          Rejects unlock when already unlocked, lock when already locked.
 *          ERROR only accepts recovery to LOCKED.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline LOCK_STATE_E lock_state_transition(LOCK_STATE_E current,
                                                  uint8_t      cmd)
{
    switch (current)
    {
        case LOCK_STATE_LOCKED:
            return (0 == cmd) ? LOCK_STATE_UNLOCKING : current;

        case LOCK_STATE_UNLOCKED:
            return (1 == cmd) ? LOCK_STATE_LOCKING : current;

        case LOCK_STATE_ERROR:
            /* safety — only allow recovery to locked */
            return (1 == cmd) ? LOCK_STATE_LOCKED : current;

        case LOCK_STATE_UNLOCKING:
        case LOCK_STATE_LOCKING:
            /* motor moving — reject all commands */
            return current;

        default:
            return LOCK_STATE_ERROR;
    }
}

/******************************************************************************
 * \brief Advance lock state after motor_off_timer confirms movement done.
 *
 * \param current - Current lock state (UNLOCKING or LOCKING).
 *
 * \return LOCK_STATE_E - Settled state after motor stops.
 *
 * \details UNLOCKING -> UNLOCKED, LOCKING -> LOCKED.
 *          All other states returned unchanged.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline LOCK_STATE_E lock_state_settle(LOCK_STATE_E current)
{
    switch (current)
    {
        case LOCK_STATE_UNLOCKING: return LOCK_STATE_UNLOCKED;
        case LOCK_STATE_LOCKING:   return LOCK_STATE_LOCKED;
        default:                   return current;
    }
}

/******************************************************************************
 * \brief Return true if lock is busy — motor is moving, commands rejected.
 *
 * \param state - Current lock state.
 *
 * \return bool - true if commands should be rejected.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline bool lock_state_is_busy(LOCK_STATE_E state)
{
    return (LOCK_STATE_UNLOCKING == state) || (LOCK_STATE_LOCKING == state);
}

/******************************************************************************
 * \brief Return Trinity log event string for a lock state transition.
 *
 * \param to - New lock state.
 *
 * \return const char* - Event string for trinity_log_event(), or NULL if
 *                       no event should be logged.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline const char *lock_state_event_str(LOCK_STATE_E to)
{
    switch (to)
    {
        case LOCK_STATE_LOCKED:    return "EVENT: LOCK\n";
        case LOCK_STATE_UNLOCKED:  return "EVENT: UNLOCK\n";
        case LOCK_STATE_UNLOCKING: return "EVENT: LOCK_UNLOCKING\n";
        case LOCK_STATE_LOCKING:   return "EVENT: LOCK_LOCKING\n";
        case LOCK_STATE_ERROR:     return "EVENT: LOCK_ERROR\n";
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
static inline const char *lock_state_label(LOCK_STATE_E state)
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
 * \brief Return the raw BLE/settings value for a settled lock state.
 *
 * \param state - Current lock state.
 *
 * \return uint8_t - 1=locked, 0=unlocked, 0xFF=transitioning/error.
 *
 * \details Used to update BLE adv mfg data and settings_save_one().
 *          Only LOCKED and UNLOCKED return meaningful values —
 *          transitional states return 0xFF sentinel.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static inline uint8_t lock_state_to_ble(LOCK_STATE_E state)
{
    switch (state)
    {
        case LOCK_STATE_LOCKED:   return 1;
        case LOCK_STATE_UNLOCKED: return 0;
        default:                  return 0xFF; /* transitioning or error */
    }
}

#endif /* LOCK_STATE_H */
