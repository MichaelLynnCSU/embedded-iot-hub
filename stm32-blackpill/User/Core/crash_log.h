#pragma once

#include <stdint.h>

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
 * fault_type:  1=HardFault, 2=MemManage, 3=BusFault, 4=UsageFault
 */
void crash_fault_handler(uint32_t *stack_frame, uint32_t fault_type);
