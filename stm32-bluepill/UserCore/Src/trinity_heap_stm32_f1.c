/******************************************************************************
 * \file trinity_heap_stm32_f1.c
 * \brief Trinity heap and task stats -- STM32F103 FreeRTOS.
 *
 * \details Uses FreeRTOS xPortGetFreeHeapSize() for heap stats.
 *          Uses vTaskGetRunTimeStats() + uxTaskGetSystemState() for task stats.
 *          Requires FreeRTOSConfig.h:
 *            configGENERATE_RUN_TIME_STATS        1
 *            configUSE_STATS_FORMATTING_FUNCTIONS  1
 *            configUSE_TRACE_FACILITY              1
 *
 *          Depends on trinity_fram_stm32_f1.c via trinity_uart_log() and
 *          panic_handler() (via trinity_fault_stm32_f1.c).
 ******************************************************************************/

#include "trinity_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#define TASK_STATS_BUF  640u

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
   static char  stats_buf[TASK_STATS_BUF];
   TaskStatus_t task_array[16];
   UBaseType_t  task_count = 0u;
   uint32_t     total_time = 0ul;
   UBaseType_t  i          = 0u;
   uint32_t     cpu_pct    = 0ul;
   char         msg[TRINITY_MSG_LEN] = {0};

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
      (void)snprintf(msg, sizeof(msg),
                     "heap exhausted: %u bytes",
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
