/******************************************************************************
 * \file trinity_private.h
 * \brief Trinity internal interface -- not for application code.
 *
 * \details Exposes symbols that must be shared across Trinity's split .c
 *          files but must never be called from main.c or application code.
 *
 *          Dependency direction (strict -- never invert):
 *            wdt    -- standalone, no Trinity deps
 *            canary -- standalone, no Trinity deps
 *            flash  -- depends on wdt (kick during scan) + canary (noinit)
 *            stats  -- depends on flash via trinity_log_event only
 *            fault  -- depends on flash via write_panic (mutex-free path)
 *            boot   -- depends on flash via trinity_log_event only
 *
 *          Rule: one owner, many readers.
 *            g_init_stage, g_canary_snapshot, g_noinit_guard -- defined in
 *            trinity_canary.c, read by trinity_flash.c and trinity_fault.c.
 *            write_panic -- defined in trinity_flash.c, called only by
 *            trinity_fault.c. Cannot use mutex (fault handler context).
 ******************************************************************************/

#ifndef TRINITY_PRIVATE_H_
#define TRINITY_PRIVATE_H_

#include <stdint.h>

/************************** CANARY GLOBALS (owned by trinity_canary.c) ********/

/* g_init_stage: last stage cookie written by main() before each risky call.
 * Also declared extern in trinity_log.h for main() writes. */
extern volatile uint32_t g_init_stage;

/* g_canary_snapshot: previous session canary saved before PRE_KERNEL_1
 * overwrites g_pre_init_canary. Read by trinity_flash.c in dump_core(). */
extern volatile uint32_t g_canary_snapshot;

/* g_noinit_guard: BSS-bleed sentinel. Read/written by trinity_flash.c
 * in dump_core(). */
extern volatile uint32_t g_noinit_guard;

/************************** FLASH INTERNALS (owned by trinity_flash.c) ********/

/**
 * \brief  Lock-free flash write -- fault handler ONLY.
 *
 * \details Bypasses g_flash_mutex. Must never be called from normal thread
 *          context. Validates TRINITY_INIT_MAGIC before touching flash to
 *          avoid nested faults if Trinity never completed init.
 *
 * \warning Calling this from any context other than k_sys_fatal_error_handler
 *          will produce data corruption or deadlock.
 */
void write_panic(const char *p_msg, uint16_t len);

#endif /* TRINITY_PRIVATE_H_ */
