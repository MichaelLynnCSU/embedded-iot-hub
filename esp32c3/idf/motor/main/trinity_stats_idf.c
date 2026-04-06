/******************************************************************************
 * \file trinity_stats_idf.c
 * \brief Trinity heap and task stats -- ESP-IDF (motor + hub).
 *
 * \details Depends on trinity_nvs_idf.c via trinity_log_event() only.
 *          No collect-then-write pattern needed -- uxTaskGetSystemState()
 *          is safe from task context, no sched-lock spinlock involved
 *          (unlike Zephyr's k_thread_foreach).
 ******************************************************************************/

#include "trinity_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define STAT_BUF_SIZE  80u

void trinity_log_heap_stats(void)
{
   size_t free_now = esp_get_free_heap_size();
   size_t free_min = esp_get_minimum_free_heap_size();
   char   msg[STAT_BUF_SIZE] = {0};

   (void)snprintf(msg, sizeof(msg),
                  "Heap: free=%u min_free=%u bytes\n",
                  (unsigned int)free_now,
                  (unsigned int)free_min);
   trinity_log_event(msg);
}

void trinity_log_task_stats(void)
{
#if (configGENERATE_RUN_TIME_STATS == 1)
   TaskStatus_t task_array[16]     = {0};
   UBaseType_t  task_count         = 0u;
   uint32_t     total_time         = 0ul;
   UBaseType_t  i                  = 0u;
   uint32_t     cpu_pct            = 0ul;
   char         msg[STAT_BUF_SIZE] = {0};

   task_count = uxTaskGetSystemState(task_array, 16u, &total_time);

   trinity_log_event("[TRINITY] Task stats:\n");

   for (i = 0u; i < task_count; i++)
   {
      cpu_pct = (total_time > 0ul)
                ? (task_array[i].ulRunTimeCounter * 100ul) / total_time
                : 0ul;

      (void)snprintf(msg, sizeof(msg),
                     "[TRINITY] %-16s cpu=%2lu%%  hwm=%u words\n",
                     task_array[i].pcTaskName,
                     (unsigned long)cpu_pct,
                     (unsigned int)task_array[i].usStackHighWaterMark);
      trinity_log_event(msg);
   }
#else
   trinity_log_event("[TRINITY] Task stats: FREERTOS_GENERATE_RUN_TIME_STATS not set\n");
#endif
}
