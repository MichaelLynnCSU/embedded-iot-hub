/******************************************************************************
 * \file trinity_log_stack.c
 * \brief trinity_log_record_low_stack() implementation.
 *
 *\details  The implementation is intentionally thin: it captures the calling
 *          task's name via pcTaskGetName(NULL), formats a fixed-width NVS-safe
 *          string, and delegates to trinity_log_event().
 *
 *          Keeping the task-name lookup here (rather than at the call-site)
 *          means the public API carries only the hwm value -- no raw pointers
 *          that could be stale or caller-supplied.
 *
 *          Units: hwm_words is ALWAYS FreeRTOS words (4 bytes each on all
 *          ESP32 variants -- Xtensa and RISC-V).  The NVS record appends 'w'
 *          as an explicit unit suffix so it is unambiguous when read back via
 *          trinity_log_dump_previous().
 *
 *          Format written to NVS (always < 64 chars):
 *            "EVENT: LOW_STACK task=<name> hwm=<n>w (<n*4>B)\n"
 ******************************************************************************/
#include "trinity_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdint.h>

/* NVS string values are capped at 4000 B in ESP-IDF but trinity_log_event()
 * typically writes short lines.  Keep well under 64 chars to be safe.       */
#define LOW_STACK_MSG_MAX  64u

void trinity_log_record_low_stack(uint32_t hwm_words)
{
    /* Resolve task name here so no raw pointer crosses the API boundary.
     * pcTaskGetName(NULL) returns a pointer to the TCB's internal name array,
     * which is always valid for the lifetime of the task.                    */
    const char *task_name = pcTaskGetName(NULL);
    char buf[LOW_STACK_MSG_MAX];

    /* Truncate task name to 15 chars (FreeRTOS configMAX_TASK_NAME_LEN - 1)
     * so the formatted string never overflows buf regardless of Kconfig.     *
     * Units: 'w' = words, bytes shown parenthetically for human readability. */
    (void)snprintf(buf, sizeof(buf),
                   "EVENT: LOW_STACK task=%.15s hwm=%luw (%luB)\n",
                   task_name ? task_name : "?",
                   (unsigned long)hwm_words,
                   (unsigned long)(hwm_words * sizeof(StackType_t)));

    trinity_log_event(buf);
}
