/******************************************************************************
 * \file trinity_fault_esp.c
 * \brief Trinity fault handler -- ESP32-C3 Zephyr (RISC-V).
 *
 * \details Overrides k_sys_fatal_error_handler(). Captures RISC-V mepc/ra
 *          and writes to flash via write_panic() (no mutex path).
 *
 *          Key differences from nRF52840 version:
 *            - RISC-V fault frame: mepc/ra instead of pc/lr
 *            - irq_lock() instead of __disable_irq()
 *            - sys_reboot(SYS_REBOOT_COLD) instead of NVIC_SystemReset()
 *            - No SCB registers (ARM-specific)
 *            - No #include <nrf.h>
 ******************************************************************************/

#include "trinity_log.h"
#include "trinity_private_esp.h"
#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/reboot.h>
#include <stdio.h>

#define CRASH_BUF_SIZE  80u

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

__attribute__((unused))
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
   char msg[CRASH_BUF_SIZE] = {0};

   if (NULL != p_esf)
   {
      /* RISC-V: mepc = machine exception PC, ra = return address */
      (void)snprintf(msg, sizeof(msg),
                     "EVENT: CRASH | TYPE: %s | PC: 0x%08X | RA: 0x%08X"
                     " | STG: 0x%04X\n",
                     reason_to_str(reason),
                     (unsigned int)p_esf->mepc,
                     (unsigned int)p_esf->ra,
                     (unsigned int)g_init_stage);
   }
   else
   {
      (void)snprintf(msg, sizeof(msg),
                     "EVENT: CRASH | TYPE: %s | PC: unknown | STG: 0x%04X\n",
                     reason_to_str(reason),
                     (unsigned int)g_init_stage);
   }

   write_panic(msg, (uint16_t)strlen(msg));

#if defined(CONFIG_TRINITY_MODE_BENCH)
   printk("\n[TRINITY] === FAULT HALT ===\n");
   printk("[TRINITY] %s", msg);
   printk("[TRINITY] STG: 0x%04X -- %s\n",
          (unsigned int)g_init_stage, stage_to_str(g_init_stage));
   printk("[TRINITY] === HALTED -- attach OpenOCD ===\n");
   irq_lock();
   while (1) {}

#elif defined(CONFIG_TRINITY_MODE_USB)
   printk("[TRINITY] %s", msg);
   k_sleep(K_MSEC(500));
   irq_lock();
   sys_reboot(SYS_REBOOT_COLD);

#else
   irq_lock();
   sys_reboot(SYS_REBOOT_COLD);
#endif
}
