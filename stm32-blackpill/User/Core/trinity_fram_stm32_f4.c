/******************************************************************************
 * \file trinity_fram_stm32_f4.c
 * \brief Trinity FRAM log -- STM32F411 bare metal.
 *
 * \details Owns FRAM access, g_fram_ok flag, trinity_uart_log(), and all
 *          log I/O. Central module -- fault, stack, and heap files all
 *          depend on this via trinity_uart_log() and trinity_rtc_store().
 *
 *          Dependency order for this platform:
 *            fram  -- standalone (owns uart_log, rtc_store)
 *            wdt   -- depends on fram (uart_log for status)
 *            fault -- depends on fram (uart_log + rtc_store)
 *            stack -- depends on fram (uart_log + panic_handler)
 *            heap  -- depends on fram (uart_log + panic_handler)
 *
 *          FRAM fail-closed: g_fram_ok = false if FRAM not detected.
 *          All FRAM write paths gated on g_fram_ok. UART continues.
 ******************************************************************************/

#include "trinity_log.h"
#include "main.h"
#include "crash_log.h"
#include "log.h"
#include "fram_driver.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define UART_TIMEOUT_MS  10u

extern UART_HandleTypeDef g_huart1;
extern I2C_HandleTypeDef  g_hi2c1;

static bool g_fram_ok = false;

/************************** UART LOG ******************************************/

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

/************************** FRAM STORE ****************************************/

void trinity_rtc_store(TRINITY_ERROR_E err)
{
   if (!g_fram_ok) { return; }
   fram_log_crash((uint8_t)err, 0u);
}

/************************** RESET CAUSE ***************************************/

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

void trinity_log_init(void)
{
   TRINITY_ERROR_E   reset_cause          = eTRINITY_ERR_NONE;
   CRASH_LOG_ENTRY_X last                 = {0};
   char              msg[TRINITY_MSG_LEN] = {0};
   uint32_t          total               = 0ul;
   uint16_t          last_addr           = 0u;
   uint16_t          entry_sz            = 0u;

   fram_init(&g_hi2c1);
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

void trinity_log_dump_previous(void)
{
   trinity_uart_log("[TRINITY] dump_previous: see trinity_log_init() output above\r\n");
}

void trinity_log_event(const char *p_msg)
{
   trinity_uart_log(p_msg);
   if (g_fram_ok)
   {
      trinity_rtc_store(eTRINITY_ERR_PANIC);
   }
}

void trinity_log_erase(void)
{
   if (!g_fram_ok) { return; }
   FRAM_META_X blank = {0};
   fram_write(FRAM_META_ADDR, (uint8_t *)&blank, sizeof(blank));
   fram_load_meta();
}
