/******************************************************************************
 * \file trinity_canary.c
 * \brief Trinity pre-init canary and noinit variables -- nRF52840 Zephyr.
 *
 * \details Fully standalone -- no dependencies on other Trinity files.
 *          Owns all .noinit variables. Registers canary_pre_init() at
 *          PRE_KERNEL_1,0 so it runs before any driver SYS_INIT can crash.
 *
 * \note    Canary snapshot fix (2026-03-26):
 *          canary_pre_init() copies g_pre_init_canary into g_canary_snapshot
 *          before overwriting with CANARY_ALIVE. dump_previous() reads
 *          g_canary_snapshot so it sees the previous session's value, not
 *          the freshly-written CANARY_ALIVE from this boot.
 *
 * \note    Stat buffer heap fix (2026-03-28):
 *          s_stat_entries as a static BSS array crossed the .noinit boundary,
 *          trampling g_init_stage and other noinit vars on every stats cycle.
 *          All noinit variables are centralized here so the linker places
 *          them contiguously and the boundary is predictable.
 *
 * \note    g_noinit_guard:
 *          Set to 0xDEADBEEF on first valid boot. If read as 0 on a
 *          non-cold boot, BSS zeroing has overrun the .noinit boundary.
 ******************************************************************************/

#include "trinity_log.h"
#include "trinity_private.h"
#include <zephyr/init.h>

/* g_noinit_guard: BSS-bleed sentinel.
 * Read and written by trinity_flash.c in trinity_log_dump_core(). */
volatile uint32_t g_noinit_guard
    __attribute__((section(".noinit")));

/* g_init_stage: declared extern in trinity_log.h and trinity_private.h.
 * Written by main() before each risky init call. Read by trinity_flash.c
 * and trinity_fault.c. */
volatile uint32_t g_init_stage
    __attribute__((section(".noinit")));

/* g_pre_init_canary: written CANARY_ALIVE at PRE_KERNEL_1 and upgraded
 * to CANARY_BOOTED inside trinity_log_init(). Reflects CURRENT boot. */
static volatile uint32_t g_pre_init_canary
    __attribute__((section(".noinit")));

/* g_canary_snapshot: copy of g_pre_init_canary saved by canary_pre_init()
 * BEFORE overwriting with CANARY_ALIVE. Holds PREVIOUS session's value.
 * Read by trinity_flash.c in trinity_log_dump_core(). */
volatile uint32_t g_canary_snapshot
    __attribute__((section(".noinit")));

/**
 * \brief  Write alive canary at earliest possible boot point.
 *
 * \details Runs at PRE_KERNEL_1,0 -- before any driver SYS_INIT that could
 *          crash. Saves previous session canary into g_canary_snapshot before
 *          overwriting so dump_previous() can classify PRE vs POST crash.
 */
static int canary_pre_init(void)
{
    g_canary_snapshot = g_pre_init_canary;
    g_pre_init_canary = TRINITY_CANARY_ALIVE;
    return 0;
}
SYS_INIT(canary_pre_init, PRE_KERNEL_1, 0);

/**
 * \brief  Mark Trinity init as completed. Called by trinity_log_init().
 *
 * \details Upgrades canary from ALIVE to BOOTED so next boot's dump can
 *          classify the crash as POST_INIT rather than PRE_INIT.
 */
void trinity_canary_set_booted(void)
{
    g_pre_init_canary = TRINITY_CANARY_BOOTED;
}
