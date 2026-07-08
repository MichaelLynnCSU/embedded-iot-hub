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
 *   BKP3  — SCB->CFSR at time of fault (was: custom 1-4 fault-type code,
 *           replaced under the v2 crash contract -- CFSR's own byte fields
 *           (MMFSR/BFSR/UFSR/HFSR bits) already distinguish MemManage vs
 *           BusFault vs UsageFault natively, so the separate custom code
 *           was redundant with real hardware fault status once the
 *           contract had room to carry it)
 *
 * On HardFault:
 *   - Writes PC/LR/CFSR to backup registers
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

/* ===== RTC backend -- board-internal, NOT part of the public
 * trinity_crash.h contract. trinity_crash_store() (in
 * trinity_fram_stm32_f4.c) calls this when it determines it's running in
 * exception context (via IPSR) and must stay fast/deterministic -- no
 * I2C/FRAM access from inside a naked fault handler. Declared in
 * crash_log.h. */
void crash_log_store_rtc(const TRINITY_CRASH_RECORD_X *p_record)
{
    if (NULL == p_record) { return; }

    backup_enable();
    bkp_write(1, p_record->pc);
    bkp_write(2, p_record->lr);
    bkp_write(3, p_record->arch_data.cortex_m.cfsr);
    bkp_write(0, CRASH_SIGNATURE);  /* write signature last */
}

/* ===== Public API ===== */

void crash_log_init(void)
{
    backup_enable();

    if (bkp_read(0) != CRASH_SIGNATURE) {
        log_enqueue("[BOOT] Clean boot\r\n");
        return;
    }

    uint32_t pc   = bkp_read(1);
    uint32_t lr   = bkp_read(2);
    uint32_t cfsr = bkp_read(3);

    /* CFSR byte fields: bits[7:0]=MMFSR, [15:8]=BFSR, [25:16]=UFSR.
     * Report which sub-register is non-zero rather than re-deriving a
     * single fault name -- a fault can set bits in more than one. */
    char buf[80];
    snprintf(buf, sizeof(buf),
             "[CRASH] CFSR=0x%08lx (MM=%02lx BF=%02lx UF=%04lx) PC=0x%08lx LR=0x%08lx\r\n",
             cfsr, (cfsr & 0xFFUL), ((cfsr >> 8) & 0xFFUL), ((cfsr >> 16) & 0x3FFUL),
             pc, lr);
    log_enqueue(buf);

    /* Clear signature — next boot is clean */
    bkp_write(0, 0x00000000UL);
}

/* ===== Fault handler core — called from naked ASM stubs in stm32f4xx_it.c ===== */
void crash_fault_handler(uint32_t *stack_frame, uint32_t fault_type)
{
    TRINITY_CRASH_RECORD_X record = {0};

    (void)fault_type;  /* superseded by real SCB->CFSR captured below */

    record.magic        = TRINITY_CRASH_MAGIC;
    record.version       = TRINITY_CRASH_VERSION;
    record.arch          = TRINITY_ARCH_CORTEX_M;
    record.error         = eTRINITY_ERR_HARDFAULT;
    record.reset_reason  = 0u;  /* not currently captured -- reserved */
    record.pc            = stack_frame[6];  /* stacked PC (index 6) */
    record.lr             = stack_frame[5];  /* stacked LR (index 5) */
    record.arch_data.cortex_m.cfsr  = SCB->CFSR;
    record.arch_data.cortex_m.hfsr  = 0u;  /* not yet captured -- reserved */
    record.arch_data.cortex_m.dfsr  = 0u;
    record.arch_data.cortex_m.afsr  = 0u;
    record.arch_data.cortex_m.mmfar = 0u;
    record.arch_data.cortex_m.bfar  = 0u;
    record.arch_data.cortex_m.icsr  = 0u;
    record.arch_data.cortex_m.shcsr = 0u;

    trinity_crash_store(&record);

#if defined(CRASH_HALT) && CRASH_HALT
    __disable_irq();
    while (1) {}  /* spin — attach SWD debugger */
#else
    NVIC_SystemReset();
#endif
}
