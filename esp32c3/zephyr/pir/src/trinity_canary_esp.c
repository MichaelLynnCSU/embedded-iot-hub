/******************************************************************************
 * \file trinity_canary_esp.c
 * \brief Trinity pre-init canary and noinit variables -- ESP32-C3 Zephyr.
 *
 * \details Fully standalone -- no dependencies on other Trinity files.
 *          Owns all .noinit variables. Registers canary_pre_init() at
 *          PRE_KERNEL_1,0 so it runs before any driver SYS_INIT can crash.
 *
 * \note    On ESP32-C3 .noinit ELF section is used (same as nRF52840).
 *          The Zephyr linker script for ESP32-C3 supports .noinit.
 *          Heap-allocated stat buffer fix applies here too -- static BSS
 *          arrays can cross .noinit boundary same as nRF52840.
 ******************************************************************************/

#include "trinity_log.h"
#include "trinity_private_esp.h"
#include <zephyr/init.h>

volatile uint32_t g_noinit_guard
    __attribute__((section(".noinit")));

volatile uint32_t g_init_stage
    __attribute__((section(".noinit")));

static volatile uint32_t g_pre_init_canary
    __attribute__((section(".noinit")));

volatile uint32_t g_canary_snapshot
    __attribute__((section(".noinit")));

static int canary_pre_init(void)
{
    g_canary_snapshot = g_pre_init_canary;
    g_pre_init_canary = TRINITY_CANARY_ALIVE;
    return 0;
}
SYS_INIT(canary_pre_init, PRE_KERNEL_1, 0);

void trinity_canary_set_booted(void)
{
    g_pre_init_canary = TRINITY_CANARY_BOOTED;
}
