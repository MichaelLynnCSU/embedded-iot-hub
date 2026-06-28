/******************************************************************************
 * \file uart_lock.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-15
 *
 * \brief UART lock command adapter for BeagleBone data controller.
 *
 * \details Extracted from uart_controller.c (2026-06-15 refactor: split
 *          into transport / protocol / correlation / lock-adapter modules,
 *          no behavior change). See uart_lock.h for design rationale.
 *
 *          This module owns g_lock_state and g_lock_mutex only — the
 *          in-flight lock state needed to drive logic_lock_transition()
 *          (controller_logic.h, the authoritative state machine) before
 *          the resolved state is written back to the registry via
 *          uart_stage_lock() (cmd/uart_staging.c).
 ******************************************************************************/

#include <pthread.h>
#include "log.h"
#include "db_manager.h"
#include "uart_lock.h"
#include "controller_logic.h"
#include "uart_staging.h"

/**
 * \brief Lock state machine — owned here, protected by g_lock_mutex.
 *        state_registry owns all other sensor state. g_lock_state is
 *        separate because it drives the lock transition logic before
 *        the resolved state is written back to the registry.
 */
static LOCK_STATE_E    g_lock_state = LOCK_STATE_LOCKED;
static pthread_mutex_t g_lock_mutex = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************************
 * \brief Process an inbound LCK frame through the lock state machine.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_process_lock(int val, int batt)
{
   LOCK_STATE_E    old_state = LOCK_STATE_LOCKED;
   LOCK_STATE_E    new_state = LOCK_STATE_LOCKED;
   const char     *p_ev      = NULL;

   pthread_mutex_lock(&g_lock_mutex);

   old_state = g_lock_state;
   new_state = logic_lock_transition(old_state, val);

   if (new_state == old_state)
   {
      if (logic_lock_is_busy(old_state))
      {
         LOG("[LCK] Command rejected — motor moving (%s)",
             logic_lock_state_label(old_state));
      }
      pthread_mutex_unlock(&g_lock_mutex);
      return;
   }

   g_lock_state = new_state;
   pthread_mutex_unlock(&g_lock_mutex);

   /* Write resolved lock state into registry.
    * uart_stage_lock() acquires state_mutex internally — must be called
    * after g_lock_mutex is released to avoid holding both simultaneously. */
   uart_stage_lock((int)new_state, batt);

   p_ev = logic_lock_event_str(old_state, new_state);

   LOG("[LCK] %s -> %s (val=%d batt=%d)",
       logic_lock_state_label(old_state),
       logic_lock_state_label(new_state),
       val, batt);

   if (NULL != p_ev)
   {
      db_save_event("LCK", p_ev);
   }
}

/******************************************************************************
 * \brief Sync TCP lock state into the UART lock state machine.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_sync_lock_state(int state)
{
   pthread_mutex_lock(&g_lock_mutex);
   g_lock_state = (LOCK_STATE_E)state;
   pthread_mutex_unlock(&g_lock_mutex);
}
