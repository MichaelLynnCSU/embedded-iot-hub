/******************************************************************************
 * \file trinity_log_stm32_f1.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Trinity crash logger -- STM32F103 FreeRTOS (stm32-bluepill).
 *
 * \details Platform: STM32F103, FreeRTOS, Cortex-M3.
 *          Storage:  FRAM crash log via FRAM_* driver (survives power loss).
 *          Console:  UART1 polled.
 *          Fault:    crash_fault_handler_c() called from ASM stubs in
 *                    stm32f1xx_it.c + FreeRTOS hooks.
 *          Canary:   None (no .noinit mechanism used on this platform).
 *          WDT:      IWDG via HAL. LSI ~40kHz, prescaler 64, reload 1875 (~3s).
 *
 *          Build modes (define in CMakeLists.txt):
 *            TRINITY_MODE_BENCH -- UART + BKPT halt (ST-Link inspection)
 *            TRINITY_MODE_FIELD -- UART + auto-reset + FRAM black box
 *
 *          FreeRTOSConfig.h must set:
 *            configGENERATE_RUN_TIME_STATS        1
 *            configUSE_STATS_FORMATTING_FUNCTIONS  1
 *            configUSE_TRACE_FACILITY              1
 *            configUSE_IDLE_HOOK                   1
 *            configCHECK_FOR_STACK_OVERFLOW        2
 *            configUSE_MALLOC_FAILED_HOOK          1
 *            portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() -- enable DWT
 *            portGET_RUN_TIME_COUNTER_VALUE()          -- return DWT->CYCCNT
 *
 * \note    crash_log.c / crash_log.h removed (2026-03-28):
 *          crash_fault_handler_c() absorbed from crash_log.c into this file.
 *          crash_log.h deleted. stm32f1xx_it.c and main.c updated to include
 *          trinity_log.h instead. All crash_log_* call sites in main.c
 *          continue to work via legacy aliases in trinity_log.h.
 *
 * \note    trinity_log_init() vs trinity_log_init_ex() (2026-03-28):
 *          The common Trinity API declares trinity_log_init(void). On F103
 *          the reset reason must be captured from RCC->CSR before clearing
 *          flags. trinity_log_init_ex(reset_reason) is the real entry point.
 *          trinity_log_init() is a convenience stub that reads RCC->CSR
 *          directly -- if main() already cleared the flags, call
 *          trinity_log_init_ex() with the pre-captured value instead.
 *
 * \note    FRAM fail-closed (2026-03-28):
 *          g_fram_ok flag set false if FRAM probe fails. All FRAM write
 *          paths gated on g_fram_ok. UART logging continues in degraded mode.
 *
 * \note    WDT fed from vApplicationIdleHook (2026-03-28):
 *          If any FreeRTOS task starves the idle task for >3s the IWDG fires.
 ******************************************************************************/

#include "trinity_log.h"
#include "main.h"
#include "fram_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/******************************** CONSTANTS ***********************************/

#define UART_TIMEOUT_MS    10u

/* IWDG: STM32F103 LSI ~40kHz, prescaler 64 → ~1.6ms/tick, reload 1875 → ~3s */
#define IWDG_PRESCALER     IWDG_PRESCALER_64
#define IWDG_RELOAD        1875u

#define TASK_STATS_BUF     640u

/************************** EXTERNAL REFERENCES *******************************/

extern UART_HandleTypeDef huart1;

/************************** STATIC (PRIVATE) DATA *****************************/

static uint32_t            g_boot_count = 0ul;
static IWDG_HandleTypeDef  g_hiwdg      = {0};
static bool                g_fram_ok    = false;

/************************** STATIC (PRIVATE) FUNCTIONS ************************/

static const char *err_to_str(TRINITY_ERROR_E err)
{
   switch (err)
   {
      case eTRINITY_ERR_NONE:      { return "clean boot";        }
      case eTRINITY_ERR_HARDFAULT: { return "HardFault";         }
      case eTRINITY_ERR_STACK:     { return "stack overflow";    }
      case eTRINITY_ERR_HEAP:      { return "heap exhaustion";   }
      case eTRINITY_ERR_ASSERT:    { return "assertion failure"; }
      case eTRINITY_ERR_BROWNOUT:  { return "brown-out reset";   }
      case eTRINITY_ERR_WATCHDOG:  { return "watchdog reset";    }
      case eTRINITY_ERR_PANIC:     { return "panic";             }
      default:                     { return "unknown";           }
   }
}

static TRINITY_ERROR_E classify_reset_cause(uint32_t reset_reason)
{
   TRINITY_ERROR_E cause = eTRINITY_ERR_UNKNOWN;

   if      (0u != (reset_reason & RCC_CSR_PORRSTF))   { cause = eTRINITY_ERR_BROWNOUT; }
   else if ((0u != (reset_reason & RCC_CSR_IWDGRSTF)) ||
            (0u != (reset_reason & RCC_CSR_WWDGRSTF))) { cause = eTRINITY_ERR_WATCHDOG; }
   else if ((0u != (reset_reason & RCC_CSR_PINRSTF))  ||
            (0u != (reset_reason & RCC_CSR_SFTRSTF)))  { cause = eTRINITY_ERR_NONE;     }

   return cause;
}

/************************** PUBLIC FUNCTIONS ***********************************/

void trinity_uart_log(const char *p_msg)
{
   uint16_t len = 0u;

   if (NULL == p_msg) { return; }
   len = (uint16_t)strlen(p_msg);
   if (0u == len) { return; }

   (void)HAL_UART_Transmit(&huart1,
                             (const uint8_t *)p_msg,
                             len,
                             UART_TIMEOUT_MS);
}

void trinity_rtc_store(TRINITY_ERROR_E err, uint8_t boot_count)
{
   if (!g_fram_ok) { return; }
   FRAM_LogCrash((uint8_t)err, boot_count);
}

/**
 * \brief  Extended init with pre-captured RCC reset reason -- preferred on F103.
 *
 * \details Call from main() after capturing RCC->CSR before clearing flags:
 *            uint32_t rcc_csr = RCC->CSR;
 *            __HAL_RCC_CLEAR_RESET_FLAGS();
 *            trinity_log_init_ex(rcc_csr);
 */
void trinity_log_init_ex(uint32_t reset_reason)
{
   TRINITY_ERROR_E    reset_cause          = eTRINITY_ERR_NONE;
   CRASH_LOG_ENTRY_X  last                 = {0};
   char               msg[TRINITY_MSG_LEN] = {0};
   uint32_t           total               = 0ul;
   uint16_t           last_addr           = 0u;
   uint16_t           entry_sz            = 0u;

   extern I2C_HandleTypeDef hi2c1;
   FRAM_Init(&hi2c1);
   uint8_t probe[16] = {0};
   g_fram_ok = (HAL_OK == FRAM_Read(FRAM_META_ADDR, probe, sizeof(probe)));
   if (!g_fram_ok)
   {
      trinity_uart_log("[TRINITY] WARN: FRAM not available -- degraded mode\r\n");
   }

   reset_cause = classify_reset_cause(reset_reason);

   if (g_fram_ok)
   {
      total        = FRAM_GetTotalCrashes();
      g_boot_count = total + 1ul;
      entry_sz     = (uint16_t)sizeof(CRASH_LOG_ENTRY_X);

      if (total > 0ul)
      {
         last_addr = (uint16_t)(FRAM_CRASHLOG_ADDR +
                     (((total - 1ul) % (FRAM_CRASHLOG_SIZE / entry_sz)) * entry_sz));

         if (HAL_OK == FRAM_Read(last_addr, (uint8_t *)&last, entry_sz))
         {
            (void)snprintf(msg, sizeof(msg),
                           "[TRINITY] Boot | crashes: %lu | last: err=0x%02X t=%lums\r\n",
                           (unsigned long)total,
                           (unsigned int)last.error_code,
                           (unsigned long)last.timestamp);
         }
         else
         {
            (void)snprintf(msg, sizeof(msg),
                           "[TRINITY] Boot | crashes: %lu | FRAM read err\r\n",
                           (unsigned long)total);
         }
      }
      else
      {
         (void)snprintf(msg, sizeof(msg), "[TRINITY] Boot | clean\r\n");
      }
   }
   else
   {
      (void)snprintf(msg, sizeof(msg), "[TRINITY] Boot | FRAM unavailable\r\n");
   }

   trinity_uart_log(msg);

   if ((eTRINITY_ERR_NONE    != reset_cause) &&
       (eTRINITY_ERR_UNKNOWN != reset_cause))
   {
      (void)snprintf(msg, sizeof(msg),
                     "[TRINITY] Reset cause: %s\r\n",
                     err_to_str(reset_cause));
      trinity_uart_log(msg);
   }
}

/**
 * \brief  Common API stub -- satisfies void trinity_log_init(void) in header.
 *
 * \details Reads RCC->CSR directly. If main() has already cleared reset flags,
 *          call trinity_log_init_ex() with the pre-captured value instead.
 */
void trinity_log_init(void)
{
   trinity_log_init_ex(RCC->CSR);
}

void trinity_log_dump_previous(void)
{
   trinity_uart_log("[TRINITY] dump_previous: see trinity_log_init_ex() output\r\n");
}

void trinity_log_event(const char *p_msg)
{
   trinity_uart_log(p_msg);
}

void trinity_log_erase(void)
{
   if (!g_fram_ok) { return; }
   uint8_t blank[16] = {0};
   FRAM_Write(FRAM_META_ADDR, blank, sizeof(blank));
   FRAM_LoadMeta();
}

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

void trinity_wdt_init(void)
{
   g_hiwdg.Instance       = IWDG;
   g_hiwdg.Init.Prescaler = IWDG_PRESCALER;
   g_hiwdg.Init.Reload    = IWDG_RELOAD;

   if (HAL_OK != HAL_IWDG_Init(&g_hiwdg))
   {
      trinity_uart_log("[TRINITY] WARN: IWDG init failed\r\n");
      return;
   }

   trinity_uart_log("[TRINITY] IWDG armed (~3s timeout)\r\n");
}

void trinity_wdt_kick(void)
{
   (void)HAL_IWDG_Refresh(&g_hiwdg);
}

void trinity_log_heap_stats(void)
{
   size_t free_now = xPortGetFreeHeapSize();
   size_t free_min = xPortGetMinimumEverFreeHeapSize();
   char   msg[TRINITY_MSG_LEN] = {0};

   (void)snprintf(msg, sizeof(msg),
                  "[TRINITY] Heap: free=%u min_free=%u bytes\r\n",
                  (unsigned int)free_now,
                  (unsigned int)free_min);
   trinity_uart_log(msg);
}

void trinity_log_task_stats(void)
{
#if (configGENERATE_RUN_TIME_STATS == 1)
   static char   stats_buf[TASK_STATS_BUF];
   TaskStatus_t  task_array[16];
   UBaseType_t   task_count = 0u;
   uint32_t      total_time = 0ul;
   UBaseType_t   i          = 0u;
   uint32_t      cpu_pct    = 0ul;
   char          msg[TRINITY_MSG_LEN] = {0};

   vTaskGetRunTimeStats(stats_buf);
   trinity_uart_log("[TRINITY] Task runtime stats:\r\n");
   trinity_uart_log(stats_buf);

   task_count = uxTaskGetSystemState(task_array, 16u, &total_time);
   trinity_uart_log("[TRINITY] Task stack HWM:\r\n");

   for (i = 0u; i < task_count; i++)
   {
      cpu_pct = (total_time > 0ul)
                ? (task_array[i].ulRunTimeCounter * 100ul) / total_time
                : 0ul;

      (void)snprintf(msg, sizeof(msg),
                     "  %-16s cpu=%2lu%% hwm=%u\r\n",
                     task_array[i].pcTaskName,
                     (unsigned long)cpu_pct,
                     (unsigned int)task_array[i].usStackHighWaterMark);
      trinity_uart_log(msg);
   }
#else
   trinity_uart_log("[TRINITY] Task stats: configGENERATE_RUN_TIME_STATS not enabled\r\n");
#endif
}

void *trinity_malloc(size_t size)
{
   void *p_mem            = NULL;
   char  msg[TRINITY_MSG_LEN] = {0};

   if (0u == size) { panic_handler("trinity_malloc: zero-size", eTRINITY_ERR_HEAP); }

   p_mem = malloc(size);

   if (NULL == p_mem)
   {
      (void)snprintf(msg, sizeof(msg), "heap exhausted: %u bytes", (unsigned int)size);
      panic_handler(msg, eTRINITY_ERR_HEAP);
   }

   return p_mem;
}

void trinity_free(void **pp_ptr)
{
   if ((NULL == pp_ptr) || (NULL == *pp_ptr)) { return; }
   free(*pp_ptr);
   *pp_ptr = NULL;
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

/************************** FAULT HANDLER -- absorbed from crash_log.c ********/

/**
 * \brief  Cortex-M3 fault handler -- unpacks fault stack, logs PC/LR, resets.
 *
 * \details Called by ASM stubs in stm32f1xx_it.c for HardFault, MemManage,
 *          BusFault, and UsageFault. The ASM stub selects MSP or PSP based
 *          on EXC_RETURN bit 2, loads fault_type into R1, then branches here
 *          with a tail-call (B not BL).
 *
 *          Cortex-M3 exception frame at fault_stack:
 *            [0]=R0 [1]=R1 [2]=R2 [3]=R3 [4]=R12 [5]=LR [6]=PC [7]=xPSR
 *
 *          fault_type: 1=HardFault 2=MemManage 3=BusFault 4=UsageFault
 *
 *          Symbol name MUST remain "crash_fault_handler_c" -- the ASM stubs
 *          in stm32f1xx_it.c branch to it by name. Changing it requires
 *          updating all four ASM stubs in stm32f1xx_it.c.
 *
 *          Previously in crash_log.c. Moved here (2026-03-28).
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
