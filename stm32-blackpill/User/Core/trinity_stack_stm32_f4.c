/******************************************************************************
 * \file trinity_stack_stm32_f4.c
 * \brief Trinity stack canary -- STM32F411 bare metal.
 *
 * \details Paints STACK_CANARY words at stack base on init.
 *          Checks on every heartbeat tick -- calls panic_handler() if
 *          corrupted. Depends on trinity_fault_stm32_f4.c (panic_handler)
 *          and trinity_fram_stm32_f4.c (trinity_uart_log).
 ******************************************************************************/

#include "trinity_log.h"
#include <stdint.h>
#include <stddef.h>

#define STACK_GUARD_WORDS  4u
#define RAM_BASE           0x20000000ul
#define RAM_END            0x20020000ul

extern uint32_t _Min_Stack_Size;
extern uint32_t _estack;

static uint32_t *g_p_stack_base = NULL;

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
