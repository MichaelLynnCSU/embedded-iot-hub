#pragma once

#include <stdint.h>
#include "trinity_log.h"

/* trinity_log.h's legacy-alias section unconditionally #defines
 * crash_log_init -> trinity_log_init. crash_log_init() here is a distinct
 * function (RTC-backup-register decode on boot) from the real
 * trinity_log_init() in trinity_fram_stm32_f4.c (FRAM init) -- undef
 * immediately, before declaring it below, so this header's own
 * declaration isn't silently renamed too. */
#undef crash_log_init

/*
 * crash_log_init — call at start of main() before Log_Drain loop.
 * Checks RTC backup registers for a previous crash, logs it via
 * the USB CDC Log() ring buffer, then clears the registers.
 */
void crash_log_init(void);

/*
 * crash_fault_handler — called from naked HardFault/MemManage/BusFault/
 * UsageFault ASM stubs in stm32f4xx_it.c.
 * stack_frame: pointer to stacked exception frame (R0-R3, R12, LR, PC, xPSR)
 * fault_type:  1=HardFault, 2=MemManage, 3=BusFault, 4=UsageFault (unused
 *              now -- superseded by real SCB->CFSR, kept for the naked-asm
 *              call signature)
 */
void crash_fault_handler(uint32_t *stack_frame, uint32_t fault_type);

/*
 * crash_log_store_rtc — board-internal RTC backend for the v2 crash
 * contract. NOT part of trinity_crash.h's public API; called only by
 * trinity_crash_store() in trinity_fram_stm32_f4.c when it detects
 * exception context. Declared here (not trinity_crash.h) because it's a
 * board-specific implementation detail, not a cross-board contract.
 */
void crash_log_store_rtc(const TRINITY_CRASH_RECORD_X *p_record);
