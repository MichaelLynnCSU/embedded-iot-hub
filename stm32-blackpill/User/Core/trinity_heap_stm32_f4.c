/******************************************************************************
 * \file trinity_heap_stm32_f4.c
 * \brief Trinity heap stats and malloc wrappers -- STM32F411 bare metal.
 *
 * \details Uses newlib mallinfo() -- available because STM32CubeMX links
 *          syscalls.c which provides _sbrk(). No RTOS heap involved.
 *          trinity_log_task_stats() is a no-op stub -- no RTOS on F411.
 *          Depends on trinity_fram_stm32_f4.c (trinity_uart_log, panic_handler).
 ******************************************************************************/

#include "trinity_log.h"
#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

static size_t g_heap_min_free = SIZE_MAX;

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
   void *p_mem            = NULL;
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
