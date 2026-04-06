/*
 * crash_log.c — STM32F411 BlackPill
 *
 * Uses RTC backup registers to persist crash info across resets.
 * F411 has 20 × 32-bit backup registers accessible without full RTC init.
 *
 * Layout:
 *   BKP0  — crash signature (0xDEADBEEF if crash logged)
 *   BKP1  — stacked PC at time of fault
 *   BKP2  — stacked LR at time of fault
 *   BKP3  — fault type (1=HardFault, 2=MemManage, 3=BusFault, 4=UsageFault)
 *
 * On HardFault:
 *   - Writes PC/LR/type to backup registers
 *   - If CRASH_HALT=1: spins forever (attach SWD debugger)
 *   - Else: resets via NVIC_SystemReset()
 *
 * On boot (crash_log_init):
 *   - Checks BKP0 for signature
 *   - If found: logs via USB CDC log_enqueue() ring buffer, clears registers
 */

#include "crash_log.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#define CRASH_SIGNATURE  0xDEADBEEFUL

#include "log.h"


/* ===== Enable RTC backup domain access ===== */
static void backup_enable(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
}

/* ===== Direct backup register read/write (no RTC handle needed) ===== */
static void bkp_write(uint32_t reg, uint32_t val)
{
    /* RTC_BKPxR registers: base RTC + 0x50 + reg*4 */
    *(__IO uint32_t *)((uint32_t)RTC_BASE + 0x50U + (reg * 4U)) = val;
}

static uint32_t bkp_read(uint32_t reg)
{
    return *(__IO uint32_t *)((uint32_t)RTC_BASE + 0x50U + (reg * 4U));
}

/* ===== Public API ===== */

void crash_log_init(void)
{
    backup_enable();

    if (bkp_read(0) != CRASH_SIGNATURE) {
        log_enqueue("[BOOT] Clean boot\r\n");
        return;
    }

    uint32_t pc         = bkp_read(1);
    uint32_t lr         = bkp_read(2);
    uint32_t fault_type = bkp_read(3);

    const char *fault_name = "UNKNOWN";
    switch (fault_type) {
        case 1: fault_name = "HARDFAULT";  break;
        case 2: fault_name = "MEMMANAGE";  break;
        case 3: fault_name = "BUSFAULT";   break;
        case 4: fault_name = "USAGEFAULT"; break;
    }

    char buf[80];
    snprintf(buf, sizeof(buf),
             "[CRASH] %s PC=0x%08lx LR=0x%08lx\r\n",
             fault_name, pc, lr);
    log_enqueue(buf);

    /* Clear signature — next boot is clean */
    bkp_write(0, 0x00000000UL);
}

/* ===== Fault handler core — called from naked ASM stubs in stm32f4xx_it.c ===== */
void crash_fault_handler(uint32_t *stack_frame, uint32_t fault_type)
{
    uint32_t pc = stack_frame[6];  /* stacked PC  (index 6 in exception frame) */
    uint32_t lr = stack_frame[5];  /* stacked LR  (index 5 in exception frame) */

    backup_enable();
    bkp_write(1, pc);
    bkp_write(2, lr);
    bkp_write(3, fault_type);
    bkp_write(0, CRASH_SIGNATURE);  /* write signature last */

#if defined(CRASH_HALT) && CRASH_HALT
    __disable_irq();
    while (1) {}  /* spin — attach SWD debugger */
#else
    NVIC_SystemReset();
#endif
}
