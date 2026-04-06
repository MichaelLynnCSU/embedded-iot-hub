/******************************************************************************
 * \file trinity_log_stm32_f4.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Trinity crash logger -- STM32F411 bare metal (stm32-blackpill).
 *
 * \details Platform: STM32F411, bare metal (no RTOS), Cortex-M4.
 *          Storage:  FRAM crash log via fram_driver (survives power loss).
 *          Console:  UART1 polled (bench/field) / USB CDC queued (USB mode).
 *          Fault:    trinity_hard_fault_handler() called from HardFault_Handler.
 *          Canary:   None (no .noinit mechanism used on this platform).
 *          WDT:      IWDG via HAL. LSI ~32kHz, prescaler 32, reload 3000 (~3s).
 *
 *          Build modes (define in CMakeLists.txt):
 *            TRINITY_MODE_BENCH -- UART + BKPT halt (ST-Link inspection)
 *            TRINITY_MODE_FIELD -- UART + auto-reset + FRAM black box
 *            TRINITY_MODE_USB   -- USB CDC + auto-reset + FRAM black box
 *
 *          FRAM layout uses crash_log driver (crash_log.h / crash_log.c).
 *          Call trinity_log_init() once after HAL_Init() in main().
 *          Call trinity_watchdog_init() after trinity_log_init().
 *          Call trinity_watchdog_kick() from heartbeat_tick() every 1s.
 *          Call trinity_paint_stack() once after trinity_log_init().
 *          Call trinity_check_stack() from heartbeat_tick() every 1s.
 *
 * \note    No pre-init canary on STM32 (2026-03-28):
 *          STM32 does not use the .noinit ELF section canary mechanism.
 *          FRAM crash log is the persistence mechanism instead.
 *          Pre-init crashes are not classified -- they reset silently and
 *          the next boot's FRAM crash count increments.
 *
 * \note    FRAM fail-closed (2026-03-28):
 *          g_fram_ok flag set to false if FRAM is not detected or read fails.
 *          All FRAM write paths are gated on g_fram_ok. UART logging continues
 *          in degraded mode if FRAM is absent or has bus errors.
 *
 * \note    STM32 execution model (2026-03-28):
 *          Bare metal single-threaded loop. No RTOS, no scheduler, no
 *          concurrent tasks. Heap stats via newlib mallinfo(). No task stats.
 *          trinity_log_task_stats() is a no-op stub on this platform.
 ******************************************************************************/

#include "trinity_log.h"
#include "main.h"
#include "crash_log.h"
#include "log.h"
#include "fram_driver.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <malloc.h>
#include <stdbool.h>
#include "stm32f4xx_hal_iwdg.h"

/******************************** CONSTANTS ***********************************/

#define UART_TIMEOUT_MS     10u
#define STACK_GUARD_WORDS    4u
#define RAM_BASE            0x20000000ul
#define RAM_END             0x20020000ul

/* IWDG: LSI ~32kHz, prescaler 32 → 1ms/tick, reload 3000 → ~3s timeout */
#define IWDG_PRESCALER      IWDG_PRESCALER_32
#define IWDG_RELOAD         3000u

/************************** EXTERNAL REFERENCES *******************************/

extern UART_HandleTypeDef g_huart1;
extern uint32_t _Min_Stack_Size;
extern uint32_t _estack;

/************************** STATIC (PRIVATE) DATA *****************************/

static uint32_t           *g_p_stack_base  = NULL;
static IWDG_HandleTypeDef  g_hiwdg         = {0};
static size_t               g_heap_min_free = SIZE_MAX;
static bool                 g_fram_ok       = false;  /* FRAM availability */

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

static TRINITY_ERROR_E classify_reset_cause(void)
{
   TRINITY_ERROR_E cause = eTRINITY_ERR_UNKNOWN;

   if      (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  { cause = eTRINITY_ERR_BROWNOUT; }
   else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ||
            __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) { cause = eTRINITY_ERR_WATCHDOG; }
   else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)  ||
            __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  { cause = eTRINITY_ERR_NONE; }

   return cause;
}

/************************** PUBLIC FUNCTIONS ***********************************/

void trinity_uart_log(const char *p_msg)
{
   uint16_t len = 0u;

   if (NULL == p_msg) { return; }
   len = (uint16_t)strlen(p_msg);
   if (0u == len) { return; }

#if defined(TRINITY_MODE_USB)
   log_enqueue(p_msg);
#else
   (void)HAL_UART_Transmit(&g_huart1,
                            (const uint8_t *)p_msg,
                            len,
                            UART_TIMEOUT_MS);
#endif
}

void trinity_rtc_store(TRINITY_ERROR_E err)
{
   if (!g_fram_ok) { return; }
   fram_log_crash((uint8_t)err, 0u);
}

void trinity_log_init(void)
{
   TRINITY_ERROR_E    reset_cause          = eTRINITY_ERR_NONE;
   CRASH_LOG_ENTRY_X  last                 = {0};
   char               msg[TRINITY_MSG_LEN] = {0};
   uint32_t           total               = 0ul;
   uint16_t           last_addr           = 0u;
   uint16_t           entry_sz            = 0u;

   /* Probe FRAM -- set g_fram_ok so all write paths are gated correctly */
   fram_init(&g_hi2c1);
   // Verify FRAM is actually responding — read the metadata block
   FRAM_META_X meta = {0};
   g_fram_ok = (HAL_OK == fram_read(FRAM_META_ADDR, (uint8_t *)&meta, sizeof(meta)));
   if (!g_fram_ok)
   {
      trinity_uart_log("[TRINITY] WARN: FRAM not available -- degraded mode\r\n");
   }

   reset_cause = classify_reset_cause();
   __HAL_RCC_CLEAR_RESET_FLAGS();

   if (g_fram_ok)
   {
      total    = fram_get_total_crashes();
      entry_sz = (uint16_t)sizeof(CRASH_LOG_ENTRY_X);

      if (total > 0ul)
      {
         last_addr = (uint16_t)(FRAM_CRASHLOG_ADDR +
                     (((total - 1ul) % (FRAM_CRASHLOG_SIZE / entry_sz)) * entry_sz));

         if (HAL_OK == fram_read(last_addr, (uint8_t *)&last, entry_sz))
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

/* IDF-style dump_previous -- no canary mechanism on STM32, print FRAM history */
void trinity_log_dump_previous(void)
{
   /* On STM32 the FRAM history is read and printed in trinity_log_init().
    * This stub exists to satisfy the common API expected by the shared header.
    * Call trinity_log_init() instead -- it covers both roles on this platform. */
   trinity_uart_log("[TRINITY] dump_previous: see trinity_log_init() output above\r\n");
}

void trinity_log_event(const char *p_msg)
{
   /* UART output always -- FRAM write only if available */
   trinity_uart_log(p_msg);
   if (g_fram_ok)
   {
      trinity_rtc_store(eTRINITY_ERR_PANIC);  /* best-effort event record */
   }
}

void trinity_log_erase(void)
{
   if (!g_fram_ok) { return; }
   FRAM_META_X blank = {0};
   fram_write(FRAM_META_ADDR, (uint8_t *)&blank, sizeof(blank));
   fram_load_meta();
}

void panic_handler(const char *p_reason, TRINITY_ERROR_E err)
{
   char msg[TRINITY_MSG_LEN] = {0};

   __disable_irq();

   (void)snprintf(msg, sizeof(msg), "[PANIC] %s\r\n",
                  (NULL != p_reason) ? p_reason : "unknown reason");
   trinity_uart_log(msg);
   trinity_rtc_store(err);

#if defined(TRINITY_MODE_BENCH)
   __asm volatile ("bkpt #0");
   while (1) {}
#else
   NVIC_SystemReset();
#endif
}

void trinity_paint_stack(void)
{
   uint32_t stack_size = 0ul;
   uint32_t i          = 0ul;

   stack_size     = (uint32_t)(uintptr_t)&_Min_Stack_Size;
   g_p_stack_base = (uint32_t *)((uintptr_t)&_estack - stack_size);

   if (((uintptr_t)g_p_stack_base < RAM_BASE) ||
       ((uintptr_t)g_p_stack_base >= RAM_END))
   {
      g_p_stack_base = NULL;
      trinity_uart_log("[TRINITY] WARN: stack base OOB, canary disabled\r\n");
      return;
   }

   for (i = 0ul; i < STACK_GUARD_WORDS; i++)
   {
      g_p_stack_base[i] = STACK_CANARY;
   }
}

void trinity_check_stack(void)
{
   uint32_t i = 0ul;

   if (NULL == g_p_stack_base) { return; }

   for (i = 0ul; i < STACK_GUARD_WORDS; i++)
   {
      if (STACK_CANARY != g_p_stack_base[i])
      {
         panic_handler("stack canary corrupted", eTRINITY_ERR_STACK);
      }
   }
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
   struct mallinfo mi       = {0};
   size_t          free_now = 0u;
   char            msg[TRINITY_MSG_LEN] = {0};

   mi       = mallinfo();
   free_now = (size_t)mi.fordblks;

   if (free_now < g_heap_min_free) { g_heap_min_free = free_now; }

   (void)snprintf(msg, sizeof(msg),
                  "[TRINITY] Heap: used=%u free=%u min_free=%u bytes\r\n",
                  (unsigned int)mi.uordblks,
                  (unsigned int)free_now,
                  (unsigned int)g_heap_min_free);
   trinity_uart_log(msg);
}

/* No RTOS on F411 -- task stats not available */
void trinity_log_task_stats(void)
{
   trinity_uart_log("[TRINITY] Task stats: not available (bare metal)\r\n");
}

void *trinity_malloc(size_t size)
{
   void *p_mem              = NULL;
   char  msg[TRINITY_MSG_LEN] = {0};

   if (0u == size)
   {
      panic_handler("trinity_malloc: zero-size request", eTRINITY_ERR_HEAP);
   }

   p_mem = malloc(size);

   if (NULL == p_mem)
   {
      (void)snprintf(msg, sizeof(msg),
                     "heap exhausted: requested %u bytes",
                     (unsigned int)size);
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
