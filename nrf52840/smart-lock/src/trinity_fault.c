/******************************************************************************
 * \file trinity_fault.c
 * \brief Trinity fault handler -- nRF52840 Zephyr.
 *
 * \details Overrides k_sys_fatal_error_handler(). Captures CFSR/HFSR/
 *          MMFAR/BFAR/PC/LR/SP/cycles and writes to flash via write_panic()
 *          (mutex-free path from trinity_private.h).
 *
 *          Depends on:
 *            trinity_flash.c -- write_panic() via trinity_private.h
 *            trinity_canary.c -- g_init_stage via trinity_private.h
 *
 *          Does NOT call trinity_log_event() -- that acquires g_flash_mutex
 *          which may be held by the crashing thread.
 *
 * \note    k_cycle_get_32() in fault handler (2026-03-25):
 *          k_uptime_get_32() depends on SysTick ISR which may be dead at
 *          fault time. k_cycle_get_32() reads DWT hardware counter directly.
 *
 * \note    g_initialized_magic sentinel (2026-03-25):
 *          write_panic() validates TRINITY_INIT_MAGIC before touching flash
 *          to avoid nested faults if Trinity never completed init.
 ******************************************************************************/

#include "trinity_log.h"
#include "trinity_private.h"
#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/reboot.h>
#include <stdio.h>
#include <nrf.h>

#define CRASH_BUF_SIZE  112u

/* ARM Cortex-M4 SCB fault register addresses */
#define SCB_CFSR   (*((volatile uint32_t *)0xE000ED28))
#define SCB_HFSR   (*((volatile uint32_t *)0xE000ED2C))
#define SCB_MMFAR  (*((volatile uint32_t *)0xE000ED34))
#define SCB_BFAR   (*((volatile uint32_t *)0xE000ED38))
#define CFSR_MMARVALID  (1u << 7)
#define CFSR_BFARVALID  (1u << 15)

static const char *reason_to_str(unsigned int reason)
{
   switch (reason)
   {
      case K_ERR_CPU_EXCEPTION:  { return "CPU_EXCEPTION";  }
      case K_ERR_SPURIOUS_IRQ:   { return "SPURIOUS_IRQ";   }
      case K_ERR_STACK_CHK_FAIL: { return "STACK_OVERFLOW"; }
      case K_ERR_KERNEL_OOPS:    { return "KERNEL_OOPS";    }
      case K_ERR_KERNEL_PANIC:   { return "KERNEL_PANIC";   }
      default:                   { return "UNKNOWN";        }
   }
}

static const char *stage_to_str(uint32_t stage)
{
   switch (stage)
   {
      case TRINITY_STAGE_RESET:     { return "RESET";     }
      case TRINITY_STAGE_GPIO:      { return "GPIO";      }
      case TRINITY_STAGE_BATTERY:   { return "BATTERY";   }
      case TRINITY_STAGE_NVS_INIT:  { return "NVS_INIT";  }
      case TRINITY_STAGE_NVS_LOAD:  { return "NVS_LOAD";  }
      case TRINITY_STAGE_LOG_INIT:  { return "LOG_INIT";  }
      case TRINITY_STAGE_WDT_INIT:  { return "WDT_INIT";  }
      case TRINITY_STAGE_BT_ENABLE: { return "BT_ENABLE"; }
      case TRINITY_STAGE_BT_ADV:    { return "BT_ADV";    }
      case TRINITY_STAGE_MAIN_LOOP: { return "MAIN_LOOP"; }
      default:                      { return "UNKNOWN";   }
   }
}

void k_sys_fatal_error_handler(unsigned int reason,
                                const struct arch_esf *p_esf)
{
   uint32_t cfsr   = SCB_CFSR;
   uint32_t hfsr   = SCB_HFSR;
   uint32_t mmfar  = (cfsr & CFSR_MMARVALID) ? SCB_MMFAR : 0u;
   uint32_t bfar   = (cfsr & CFSR_BFARVALID) ? SCB_BFAR  : 0u;
   uint32_t cycles = k_cycle_get_32();

   char line1[CRASH_BUF_SIZE] = {0};
   char line2[CRASH_BUF_SIZE] = {0};

   if (NULL != p_esf)
   {
      (void)snprintf(line1, sizeof(line1),
                     "EVENT: CRASH | TYPE: %s(%u)"
                     " | PC: 0x%08X | LR: 0x%08X | SP: 0x%08X"
                     " | CYC: %u | STG: 0x%04X\n",
                     reason_to_str(reason), reason,
                     (unsigned int)p_esf->basic.pc,
                     (unsigned int)p_esf->basic.lr,
                     (unsigned int)p_esf,
                     cycles,
                     (unsigned int)g_init_stage);
   }
   else
   {
      (void)snprintf(line1, sizeof(line1),
                     "EVENT: CRASH | TYPE: %s(%u) | PC: unknown"
                     " | CYC: %u | STG: 0x%04X\n",
                     reason_to_str(reason), reason,
                     cycles,
                     (unsigned int)g_init_stage);
   }

   (void)snprintf(line2, sizeof(line2),
                  "EVENT: FAULT | CFSR: 0x%08X | HFSR: 0x%08X"
                  " | MMFAR: 0x%08X | BFAR: 0x%08X\n",
                  cfsr, hfsr, mmfar, bfar);

   write_panic(line1, (uint16_t)strlen(line1));
   write_panic(line2, (uint16_t)strlen(line2));

#if defined(CONFIG_TRINITY_MODE_BENCH)
   printk("\n[TRINITY] === FAULT HALT ===\n");
   printk("[TRINITY] %s", line1);
   printk("[TRINITY] %s", line2);
   printk("[TRINITY] STG  : 0x%04X -- %s\n",
          (unsigned int)g_init_stage, stage_to_str(g_init_stage));
   printk("[TRINITY] === HALTED -- attach JLink or read RTT ===\n");
   __disable_irq();
   while (1) {}

#elif defined(CONFIG_TRINITY_MODE_USB)
   k_sleep(K_MSEC(500));
   __disable_irq();
   NVIC_SystemReset();

#else
   __disable_irq();
   NVIC_SystemReset();
#endif
}
