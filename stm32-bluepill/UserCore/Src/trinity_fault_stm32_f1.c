/******************************************************************************
 * \file trinity_fault_stm32_f1.c
 * \brief Trinity fault handler and FreeRTOS hooks -- STM32F103 FreeRTOS.
 *
 * \details Depends on trinity_fram_stm32_f1.c via:
 *            trinity_uart_log()  -- polled UART output
 *            trinity_rtc_store() -- FRAM crash record
 *            g_boot_count        -- extern from trinity_fram_stm32_f1.c
 *
 * \warning crash_fault_handler_c() MUST keep this exact name -- the ASM
 *          stubs in stm32f1xx_it.c branch to it by name. Changing the
 *          symbol requires updating all four ASM stubs.
 *
 *          vApplicationIdleHook() kicks the WDT -- trinity_wdt_kick() must
 *          be available (trinity_wdt_stm32_f1.c or its stub).
 ******************************************************************************/

#include "trinity_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/* g_boot_count defined in trinity_fram_stm32_f1.c */
extern uint32_t g_boot_count;

/************************** PANIC / HARD FAULT ********************************/

void panic_handler(const char *p_reason, TRINITY_ERROR_E err)
{
   char msg[TRINITY_MSG_LEN] = {0};

   taskDISABLE_INTERRUPTS();

   (void)snprintf(msg, sizeof(msg), "[PANIC] %s\r\n",
                  (NULL != p_reason) ? p_reason : "unknown reason");
   trinity_uart_log(msg);
   trinity_rtc_store(err, (uint8_t)g_boot_count);

#if defined(TRINITY_MODE_BENCH)
   __asm volatile ("bkpt #0");
   while (1) {}
#else
   NVIC_SystemReset();
#endif
}

void trinity_hard_fault_handler(void)
{
   trinity_uart_log("[TRINITY] HardFault!\r\n");
   trinity_rtc_store(eTRINITY_ERR_HARDFAULT, (uint8_t)g_boot_count);

#if defined(TRINITY_MODE_BENCH)
   __asm volatile ("bkpt #0");
   while (1) {}
#else
   NVIC_SystemReset();
#endif
}

/************************** CORTEX-M3 FAULT HANDLER ***************************/

/**
 * \brief  Cortex-M3 fault handler -- unpacks fault stack, logs PC/LR, resets.
 *
 * \details Called by ASM stubs in stm32f1xx_it.c for HardFault, MemManage,
 *          BusFault, UsageFault. The ASM stub selects MSP or PSP based on
 *          EXC_RETURN bit 2, loads fault_type into R1, then branches here.
 *
 *          Cortex-M3 exception frame:
 *            [0]=R0 [1]=R1 [2]=R2 [3]=R3 [4]=R12 [5]=LR [6]=PC [7]=xPSR
 *
 *          fault_type: 1=HardFault 2=MemManage 3=BusFault 4=UsageFault
 *
 * \warning Symbol name MUST remain "crash_fault_handler_c". ASM stubs in
 *          stm32f1xx_it.c branch to it by name. Do not rename.
 */
void crash_fault_handler_c(uint32_t *fault_stack, uint32_t fault_type)
{
#if defined(TRINITY_MODE_BENCH)
   (void)fault_stack;
   (void)fault_type;
   __disable_irq();
   while (1) {}

#else
   char        buf[80]    = {0};
   uint32_t    pc         = fault_stack[6];
   uint32_t    lr         = fault_stack[5];
   const char *fault_name = "FAULT";

   switch (fault_type)
   {
      case 1u: { fault_name = "HARDFAULT";  break; }
      case 2u: { fault_name = "MEMMANAGE";  break; }
      case 3u: { fault_name = "BUSFAULT";   break; }
      case 4u: { fault_name = "USAGEFAULT"; break; }
      default: { break; }
   }

   trinity_uart_log("\r\n[TRINITY] === FAULT ===\r\n");

   (void)snprintf(buf, sizeof(buf), "[TRINITY] TYPE: %s\r\n", fault_name);
   trinity_uart_log(buf);

   (void)snprintf(buf, sizeof(buf), "[TRINITY] PC = 0x%08lX\r\n", (unsigned long)pc);
   trinity_uart_log(buf);

   (void)snprintf(buf, sizeof(buf), "[TRINITY] LR = 0x%08lX\r\n", (unsigned long)lr);
   trinity_uart_log(buf);

   (void)snprintf(buf, sizeof(buf),
                  "EVENT: %s | PC=0x%08lX | LR=0x%08lX\n",
                  fault_name, (unsigned long)pc, (unsigned long)lr);
   trinity_log_event(buf);
   trinity_rtc_store(eTRINITY_ERR_HARDFAULT, (uint8_t)g_boot_count);

   NVIC_SystemReset();
#endif
}

/** Legacy alias -- some projects call HardFault_Handler_C directly */
void HardFault_Handler_C(uint32_t *fault_stack)
{
   crash_fault_handler_c(fault_stack, 1u);
}

/************************** FREERTOS HOOKS ************************************/

void vApplicationStackOverflowHook(TaskHandle_t p_task, char *p_task_name)
{
   char msg[TRINITY_MSG_LEN] = {0};

   (void)p_task;

   (void)snprintf(msg, sizeof(msg),
                  "[TRINITY] Stack overflow: %s\r\n",
                  (NULL != p_task_name) ? p_task_name : "unknown");
   trinity_uart_log(msg);
   trinity_rtc_store(eTRINITY_ERR_STACK, (uint8_t)g_boot_count);

#if defined(TRINITY_MODE_BENCH)
   __asm volatile ("bkpt #0");
   while (1) {}
#else
   NVIC_SystemReset();
#endif
}

void vApplicationMallocFailedHook(void)
{
   trinity_uart_log("[TRINITY] FreeRTOS heap allocation failed!\r\n");
   trinity_rtc_store(eTRINITY_ERR_HEAP, (uint8_t)g_boot_count);

#if defined(TRINITY_MODE_BENCH)
   __asm volatile ("bkpt #0");
   while (1) {}
#else
   NVIC_SystemReset();
#endif
}

void vApplicationIdleHook(void)
{
   trinity_wdt_kick();
}
