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

/* This file is shared between motor (RISC-V, esp32c3) and hub (Xtensa,
 * esp32) -- see file \brief above. RvExcFrame is RISC-V-only; including
 * it unconditionally broke the hub build (riscv component isn't, and
 * shouldn't be, a dependency of an Xtensa target). Guard on ESP-IDF's
 * own target-arch Kconfig macro so this header/cast only compiles in
 * for the RISC-V build. */
#if defined(CONFIG_IDF_TARGET_ARCH_RISCV) && CONFIG_IDF_TARGET_ARCH_RISCV
#include "riscv/rvruntime-frames.h"
#endif

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

   /* v2 crash contract: additionally build the canonical record and call
    * trinity_crash_store() (trinity_nvs_idf.c). Additive -- does not
    * replace the trinity_log_event() lines above.
    *
    * arch differs by which real target this file is compiled for --
    * motor (esp32c3) is RISC-V, hub (esp32) is Xtensa. Only the RISC-V
    * build parses RvExcFrame (verified against this project's actual
    * struct, components/riscv/include/riscv/rvruntime-frames.h:
    * mepc/mstatus/mcause/mtval all real members). The Xtensa build gets
    * TRINITY_ARCH_XTENSA with arch_data left reserved/zero -- Xtensa
    * frame parsing is out of scope for this pass; xtensa's own
    * XtExcFrame equivalent hasn't been verified yet. */
   TRINITY_CRASH_RECORD_X record = {0};
   record.magic        = TRINITY_CRASH_MAGIC;
   record.version      = TRINITY_CRASH_VERSION;
#if defined(CONFIG_IDF_TARGET_ARCH_RISCV) && CONFIG_IDF_TARGET_ARCH_RISCV
   record.arch         = TRINITY_ARCH_RISCV;
#else
   record.arch         = TRINITY_ARCH_XTENSA;
#endif
   record.error        = eTRINITY_ERR_HARDFAULT;
   record.reset_reason = 0u;
   record.pc           = (NULL != p_info) ? (uint32_t)(uintptr_t)p_info->addr : 0u;
   record.lr           = 0u;
#if defined(CONFIG_IDF_TARGET_ARCH_RISCV) && CONFIG_IDF_TARGET_ARCH_RISCV
   if ((NULL != p_info) && (NULL != p_info->frame))
   {
      const RvExcFrame *p_frame = (const RvExcFrame *)p_info->frame;
      record.arch_data.riscv.mepc    = (uint32_t)p_frame->mepc;
      record.arch_data.riscv.mstatus = (uint32_t)p_frame->mstatus;
      record.arch_data.riscv.mcause  = (uint32_t)p_frame->mcause;
      record.arch_data.riscv.mtval   = (uint32_t)p_frame->mtval;
   }
#endif
   trinity_crash_store(&record);

#if defined(CONFIG_TRINITY_MODE_USB) && CONFIG_TRINITY_MODE_USB
   vTaskDelay(pdMS_TO_TICKS(500));
#endif

   esp_restart();
#endif
}
