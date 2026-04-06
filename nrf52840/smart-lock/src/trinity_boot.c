/******************************************************************************
 * \file trinity_boot.c
 * \brief Trinity boot reason decoder -- nRF52840 Zephyr.
 *
 * \details Centralizes NRF_POWER->RESETREAS decode so all nRF52840 boards
 *          get identical boot reason logging. Depends on trinity_flash.c
 *          via trinity_log_event() only.
 *
 *          main() must read and clear RESETREAS at the very top before
 *          anything else, then call trinity_log_boot_reason() after
 *          trinity_log_init():
 *
 *            uint32_t reset_reason = NRF_POWER->RESETREAS;
 *            NRF_POWER->RESETREAS  = 0xFFFFFFFF;
 *            ...
 *            if (trinity_log_init() == 0) {
 *                trinity_log_boot_reason(reset_reason);
 *            }
 *
 * \note    RESETREAS is a latched OR-history register -- NOT auto-cleared
 *          on reset. Must be read and cleared before any reset can fire,
 *          otherwise stale bits pollute future boot logs.
 *          Combined causes (e.g. DOG|REPOR on first post-flash boot) are
 *          always logged as raw hex alongside the decoded label.
 ******************************************************************************/

#include "trinity_log.h"
#include <stdio.h>

/* nRF52840 RESETREAS bit definitions -- private to this file */
#define TRINITY_RESET_PIN       (1u << 0)  /* RESETPIN */
#define TRINITY_RESET_DOG       (1u << 1)  /* watchdog */
#define TRINITY_RESET_SREQ      (1u << 2)  /* soft reset */
#define TRINITY_RESET_LOCKUP    (1u << 3)  /* CPU lockup */
#define TRINITY_RESET_REPOR     (1u << 16) /* cold POR / OFF */

void trinity_log_boot_reason(uint32_t resetreas)
{
   const char *label   = "NONE";
   char        buf[80] = {0};

   if      (resetreas & TRINITY_RESET_DOG)    { label = "WATCHDOG";   }
   else if (resetreas & TRINITY_RESET_LOCKUP) { label = "LOCKUP";     }
   else if (resetreas & TRINITY_RESET_SREQ)   { label = "SOFT_RESET"; }
   else if (resetreas & TRINITY_RESET_PIN)    { label = "RESET_PIN";  }
   else if (resetreas & TRINITY_RESET_REPOR)  { label = "COLD_POR";   }

   (void)snprintf(buf, sizeof(buf),
                  "EVENT: BOOT | RESETREAS: 0x%08X | REASON: %s\n",
                  resetreas, label);
   trinity_log_event(buf);
}

TRINITY_BOOT_REASON_E trinity_classify_reset(uint32_t resetreas)
{
    if (resetreas & TRINITY_RESET_DOG)   { return TRINITY_BOOT_WATCHDOG;   }
    if (resetreas & TRINITY_RESET_SREQ)  { return TRINITY_BOOT_SOFT_RESET; }
    if (resetreas & TRINITY_RESET_PIN)   { return TRINITY_BOOT_RESET_PIN;  }
    if (resetreas & TRINITY_RESET_REPOR) { return TRINITY_BOOT_UNKNOWN;    }
    if (g_noinit_guard    == 0xDEADBEEF &&
        g_canary_snapshot == TRINITY_CANARY_BOOTED)
    {
        return TRINITY_BOOT_BROWNOUT;
    }
    return TRINITY_BOOT_COLD_POWER_ON;
}
