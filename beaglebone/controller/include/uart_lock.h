/******************************************************************************
 * \file uart_lock.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-15
 *
 * \brief UART lock command adapter for BeagleBone data controller.
 *
 * \details Extracted from uart_controller.c as part of the
 *          transport/protocol/correlation/lock-adapter split (no behavior
 *          change). This module is a thin adapter only — it holds no lock
 *          state-transition logic. All transitions are validated by
 *          logic_lock_transition() (controller_logic.h), the authoritative
 *          state machine. uart_lock.c owns only g_lock_state / g_lock_mutex,
 *          the in-flight value needed to drive that state machine and
 *          resolve the result back into state_registry via
 *          uart_stage_lock().
 *
 *          Dependency direction:
 *            uart_transport.c -> uart_lock.c -> controller_logic.c (state machine)
 *                                             -> uart_staging.c (registry write)
 *
 *          g_lock_mutex and state_registry's internal state_mutex are never
 *          held simultaneously — no ABBA deadlock risk (see 2026-06-09 note
 *          preserved from uart_controller.c).
 ******************************************************************************/
#ifndef INCLUDE_UART_LOCK_H_
#define INCLUDE_UART_LOCK_H_

/******************************************************************************
 * \brief Process an inbound LCK frame through the lock state machine.
 *
 * \param val  - Parsed lock value from inbound UART frame (0=lock, 1=unlock).
 * \param batt - Battery SOC percent, or -1 if absent.
 *
 * \return void
 *
 * \details g_lock_state is guarded by g_lock_mutex (private to this module).
 *          After resolving the new lock state via logic_lock_transition(),
 *          the result is written into the registry via uart_stage_lock(),
 *          which acquires state_mutex internally. The two mutexes are never
 *          held simultaneously.
 ******************************************************************************/
void uart_process_lock(int val, int batt);

/******************************************************************************
 * \brief Sync TCP lock state into the UART lock state machine.
 *
 * \param state - New lock state value to set.
 *
 * \return void
 *
 * \details Guards g_lock_state with g_lock_mutex. state_registry owns all
 *          other sensor state — g_lock_state is separate because it drives
 *          the lock transition logic before the resolved state is written
 *          back to the registry via uart_stage_lock().
 ******************************************************************************/
void uart_sync_lock_state(int state);

#endif /* INCLUDE_UART_LOCK_H_ */
