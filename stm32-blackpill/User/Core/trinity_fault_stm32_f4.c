/******************************************************************************
 * \file trinity_fault_stm32_f4.c
 * \brief Trinity fault and panic handler -- STM32F411 bare metal.
 *
 * \details Depends on trinity_fram_stm32_f4.c via:
 *            trinity_uart_log()  -- polled UART output
 *            trinity_rtc_store() -- FRAM crash record
 ******************************************************************************/

#include "trinity_log.h"
#include "crash_log.h"
#include "stm32f4xx_hal.h" 
#include <stdio.h>
#include <string.h>

void panic_handler(const char *p_reason, TRINITY_ERROR_E err)
{
   char                    msg[TRINITY_MSG_LEN] = {0};
   TRINITY_CRASH_RECORD_X  record                = {0};

   __disable_irq();

   (void)snprintf(msg, sizeof(msg), "[PANIC] %s\r\n",
                  (NULL != p_reason) ? p_reason : "unknown reason");
   trinity_uart_log(msg);

   record.magic         = TRINITY_CRASH_MAGIC;
   record.version       = TRINITY_CRASH_VERSION;
   record.arch          = TRINITY_ARCH_CORTEX_M;
   record.error         = err;
   record.reset_reason  = 0u;
   record.pc            = 0u;  /* not available at this call site, as before */
   record.lr            = 0u;
   /* arch_data left zeroed -- not available/relevant at a software panic
    * call site (no hardware fault occurred here). */
   trinity_crash_store(&record);

#if defined(TRINITY_MODE_BENCH)
   __asm volatile ("bkpt #0");
   while (1) {}
#else
   NVIC_SystemReset();
#endif
}

void trinity_hard_fault_handler(void)
{
   crash_log_init();
   trinity_uart_log("[TRINITY] HardFault!\r\n");
   trinity_rtc_store(eTRINITY_ERR_HARDFAULT);

#if defined(TRINITY_MODE_BENCH)
   __asm volatile ("bkpt #0");
   while (1) {}
#else
   NVIC_SystemReset();
#endif
}
