/******************************************************************************
 * \file trinity_stats_esp.c
 * \brief Trinity heap and task stats -- ESP32-C3 Zephyr.
 *
 * \details Identical logic to nRF52840 version. Depends on
 *          trinity_flash_esp.c via trinity_log_event() only.
 *          Heap-allocated stat buffer to avoid .noinit overlap.
 ******************************************************************************/

#include "trinity_log.h"
#include <zephyr/kernel.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/sys/mem_stats.h>
#include <stdio.h>

#define STAT_BUF_SIZE    80u
#define MAX_STAT_THREADS 12

typedef struct { char msg[STAT_BUF_SIZE]; } STAT_ENTRY_T;

static STAT_ENTRY_T *s_stat_entries = NULL;
static int           s_stat_count   = 0;

int trinity_log_stats_init(void)
{
   s_stat_entries = k_malloc(sizeof(STAT_ENTRY_T) * MAX_STAT_THREADS);
   if (NULL == s_stat_entries)
   {
      printk("[TRINITY] FATAL: stat buffer alloc failed -- thread stats disabled\n");
      return -ENOMEM;
   }
   return 0;
}

void trinity_log_heap_stats(void)
{
#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
   extern struct k_heap _system_heap;
   struct sys_memory_stats stats = {0};
   char                    msg[STAT_BUF_SIZE];
   int                     rc    = 0;

   rc = sys_heap_runtime_stats_get(&_system_heap.heap, &stats);
   if (0 != rc) { printk("[TRINITY] Heap stats unavailable\n"); return; }

   (void)snprintf(msg, sizeof(msg),
                  "Heap: free=%zu allocated=%zu max_allocated=%zu\n",
                  stats.free_bytes,
                  stats.allocated_bytes,
                  stats.max_allocated_bytes);
   trinity_log_event(msg);
#else
   trinity_log_event("Heap stats: CONFIG_SYS_HEAP_RUNTIME_STATS not set\n");
#endif
}

static void trinity_thread_cb_collect(struct thread_analyzer_info *p_info)
{
   if (NULL == s_stat_entries)            { return; }
   if (s_stat_count >= MAX_STAT_THREADS)  { return; }

   (void)snprintf(s_stat_entries[s_stat_count].msg, STAT_BUF_SIZE,
                  "[TRINITY] Thread: %-16s cpu=%3u%%  stack=%u/%u\n",
                  p_info->name,
                  p_info->utilization,
                  p_info->stack_used,
                  p_info->stack_size);
   s_stat_count++;
}

void trinity_log_task_stats(void)
{
   int i;

   if (NULL == s_stat_entries) { return; }

   trinity_wdt_kick();
   s_stat_count = 0;
   thread_analyzer_run(trinity_thread_cb_collect, 0);

   trinity_wdt_kick();
   trinity_log_event("[TRINITY] Thread stats:\n");

   for (i = 0; i < s_stat_count; i++)
   {
      trinity_log_event(s_stat_entries[i].msg);
   }
   trinity_wdt_kick();
}
