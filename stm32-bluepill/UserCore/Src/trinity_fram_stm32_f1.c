/******************************************************************************
 * Copyright (c) 2025 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    trinity_fram_stm32_f1.c
 * \author  MichaelLynnCSU
 * \date    01-01-2025
 *
 * \brief   Trinity FRAM log — STM32F103 (BluePill).
 *
 * \details Owns FRAM access, g_fram_ok, g_boot_count, trinity_uart_log(),
 *          trinity_rtc_store(), and all log I/O.  Central module on F103 —
 *          all other files depend on this via uart_log and rtc_store.
 *
 *          g_boot_count is shared with trinity_fault_stm32_f1.c via extern.
 *          It is incremented here in trinity_log_init_ex() and read in
 *          panic_handler() and FreeRTOS hooks.
 *
 *          trinity_log_init_ex() is the preferred entry point — main() must
 *          capture RCC->CSR before clearing flags, then call init_ex() with
 *          the captured value.  trinity_log_init() is a convenience stub
 *          that reads RCC->CSR directly — use only if flags have not been
 *          cleared yet.
 *
 * \note    Crash-log state has moved to ring_buffer.c.
 *          Call sequence inside trinity_log_init_ex():
 *            1. FRAM_Init()         — stores I2C handle, no meta load
 *            2. FRAM_Read() probe   — confirms FRAM is alive
 *            3. rb_crashlog_init()  — restores crash write-ptr + total from
 *                                     FRAM meta (only if g_fram_ok)
 *          trinity_rtc_store() calls rb_crashlog_push() (was FRAM_LogCrash).
 *          trinity_log_erase() calls rb_crashlog_init() to reset RAM state
 *          after blanking the meta block (was FRAM_LoadMeta).
 ******************************************************************************/

#include "trinity_log.h"
#include "ring_buffer.h"
#include "fram_driver.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define UART_TIMEOUT_MS  10u

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef  hi2c1;

static bool g_fram_ok = false;

/* Shared with trinity_fault_stm32_f1.c — used in rtc_store calls */
uint32_t g_boot_count = 0ul;

/************************** UART LOG ******************************************/

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

/************************** FRAM STORE ****************************************/

/**
 * \brief  Persist a crash event to the FRAM crash partition.
 *
 * \details Delegates to rb_crashlog_push() — ring_buffer.c now owns the
 *          crash write pointer and entry count (was FRAM_LogCrash()).
 *
 * \param  err        - Error code to record.
 * \param  boot_count - Current boot counter.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void trinity_rtc_store(TRINITY_ERROR_E err, uint8_t boot_count)
{
   if (!g_fram_ok) { return; }
   rb_crashlog_push((uint8_t)err, boot_count);
}

/************************** RESET CAUSE ***************************************/

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

/************************** PUBLIC API ****************************************/

/**
 * \brief  Initialise the trinity log subsystem.
 *
 * \details Call sequence:
 *            1. FRAM_Init()         — stores hi2c1, no meta read
 *            2. Probe FRAM_Read()   — sets g_fram_ok
 *            3. rb_crashlog_init()  — restores crash state from FRAM meta
 *               (only executed when g_fram_ok, skipped in degraded mode)
 *            4. Read last crash entry for boot message
 *            5. Log reset cause if non-trivial
 *
 * \param  reset_reason - Value of RCC->CSR captured before flag clear.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void trinity_log_init_ex(uint32_t reset_reason)
{
   TRINITY_ERROR_E   reset_cause          = eTRINITY_ERR_NONE;
   CRASH_LOG_ENTRY_X last                 = {0};
   char              msg[TRINITY_MSG_LEN] = {0};
   uint32_t          total                = 0ul;
   uint16_t          last_addr            = 0u;
   uint16_t          entry_sz             = 0u;

   /* --- 1. Initialise driver (handle only, no meta load) --- */
   FRAM_Init(&hi2c1);

   /* --- 2. Probe FRAM --- */
   uint8_t probe[16] = {0};
   g_fram_ok = (HAL_OK == FRAM_Read(FRAM_META_ADDR, probe, sizeof(probe)));

   if (!g_fram_ok)
   {
      trinity_uart_log("[TRINITY] WARN: FRAM not available -- degraded mode\r\n");
   }

   /* --- 3. Restore crash-log RAM state from FRAM meta --- */
   if (g_fram_ok)
   {
      rb_crashlog_init();
   }

   /* --- 4. Classify reset cause --- */
   reset_cause = classify_reset_cause(reset_reason);

   /* --- 5. Compose boot message --- */
   if (g_fram_ok)
   {
      total        = rb_crashlog_get_total();
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

   /* --- 6. Log reset cause if non-trivial --- */
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
 * \brief  Convenience wrapper — reads RCC->CSR directly.
 *
 * \details Use only if the reset flags have not yet been cleared.
 *
 * \return void
 *
 * \author MichaelLynnCSU
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

/**
 * \brief  Erase all FRAM log data and reset RAM state.
 *
 * \details Blanks the meta block, then calls rb_crashlog_init() to reload
 *          the (now-zeroed) crash fields into ring_buffer.c's static
 *          variables.  Previously called FRAM_LoadMeta() directly, which
 *          no longer exists in the zero-arg form.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void trinity_log_erase(void)
{
   uint8_t blank[16] = {0};
   if (!g_fram_ok) { return; }
   FRAM_Write(FRAM_META_ADDR, blank, sizeof(blank));
   rb_crashlog_init();   /* re-read zeroed meta → resets g_crash_write_ptr
                          * and g_total_crash_entries in ring_buffer.c      */
}
