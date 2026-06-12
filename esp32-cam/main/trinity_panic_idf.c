/******************************************************************************
 * \file trinity_panic_idf.c
 * \brief Trinity panic handler -- ESP-IDF (motor + hub).
 *
 * \details Overrides esp_panic_handler_user(). Calls trinity_log_event()
 *          directly -- no write_panic needed on IDF since the panic handler
 *          can take the NVS mutex (unlike Zephyr's fault handler which runs
 *          in a context where mutex acquisition would deadlock).
 *
 *          Called by ESP-IDF on:
 *            - Unhandled exceptions (load/store fault, illegal instruction)
 *            - Task WDT timeout (CONFIG_ESP_TASK_WDT_PANIC=y required)
 *            - Interrupt WDT timeout
 *            - FreeRTOS stack overflow
 *            - abort() and assert() failures
 ******************************************************************************/

#include "trinity_log.h"
#include "esp_system.h"
#include "esp_private/panic_internal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define CRASH_BUF_SIZE  64

void esp_panic_handler_user(panic_info_t *p_info)
{
#if defined(CONFIG_TRINITY_MODE_BENCH) && CONFIG_TRINITY_MODE_BENCH
   while (1) {}

#else
   char buf[CRASH_BUF_SIZE] = {0};

   if ((NULL != p_info) && (NULL != p_info->addr))
   {
      (void)snprintf(buf, sizeof(buf),
                     "EVENT: CRASH | PC: 0x%08lx\n",
                     (unsigned long)(uintptr_t)p_info->addr);
      trinity_log_event(buf);
   }
   else
   {
      trinity_log_event("EVENT: CRASH | PC: unknown\n");
   }

#if defined(CONFIG_TRINITY_MODE_USB) && CONFIG_TRINITY_MODE_USB
   vTaskDelay(pdMS_TO_TICKS(500));
#endif

   esp_restart();
#endif
}
