/******************************************************************************
 * \file    trinity_crash.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief   Unified multi-architecture crash record contract -- all Trinity
 *          boards wired today: STM32 (F411, F103), nRF52840 (smart-light,
 *          smart-lock, reed-sensor, temp-sensor), ESP32-C3 (motor + pir),
 *          and ESP32/ESP32-S3 (hub, cam, doorbell -- arch_data.xtensa still
 *          reserved, see \note below).
 *
 * \details v2 of the crash contract. v1 (uint8_t error_code/boot_count,
 *          20-byte fixed layout, split trinity_crash_store_rtc()/
 *          trinity_crash_store_fram()) is superseded by this version:
 *          - error is now TRINITY_ERROR_E directly, not a raw uint8_t.
 *          - arch is a real architecture family (TRINITY_ARCH_E), not a
 *            per-board-model enum.
 *          - arch_data is a union of raw architecture fault-status
 *            registers, room for actually capturing them instead of
 *            leaving them out.
 *          - single trinity_crash_store(record) entry point per board,
 *            instead of separate _rtc/_fram functions. The board decides
 *            internally how the record becomes persistent -- see each
 *            board's .c file for how it picks a backend.
 *          - boot_count and timestamp are dropped from the canonical
 *            record. Both were already board/backend-specific bookkeeping
 *            (boot_count from a board global, timestamp overwritten by the
 *            FRAM ring buffer itself) rather than genuine fault-context
 *            data, so they don't belong in the architecture-neutral
 *            contract. A backend that wants them reads them itself at
 *            store time rather than expecting the caller to supply them.
 *
 * \note    SCOPE (updated as each arch/board is actually wired --
 *          keep this current, not aspirational):
 *
 *          TRINITY_ARCH_CORTEX_M -- capture depth differs by board, not
 *          uniform across the arch:
 *            - F411, F103 (STM32): only cfsr populated (real SCB->CFSR).
 *              hfsr/dfsr/afsr/mmfar/bfar/icsr/shcsr remain reserved/zero --
 *              not yet captured by either board's fault handler.
 *            - smart-light, smart-lock, reed-sensor, temp-sensor (nRF52840,
 *              Zephyr k_sys_fatal_error_handler): cfsr/hfsr/mmfar/bfar all
 *              populated -- richer than STM32's capture today. dfsr/afsr/
 *              icsr/shcsr remain reserved/zero, same as STM32.
 *
 *          TRINITY_ARCH_RISCV -- ESP32-C3, two independent targets,
 *          populated differently per platform because the underlying
 *          exception-frame structs differ:
 *            - IDF (motor's build of the shared trinity_panic_idf.c also
 *              used by hub/cam/doorbell -- RISC-V build only, selected via
 *              CONFIG_IDF_TARGET_ARCH_RISCV): all four fields populated --
 *              mepc/mstatus/mcause/mtval -- parsed from the real
 *              RvExcFrame (components/riscv/include/riscv/
 *              rvruntime-frames.h).
 *            - Zephyr (pir): only mepc/mstatus populated. mcause is
 *              absent from struct arch_esf unless
 *              CONFIG_CLIC_SUPPORT_INTERRUPT_LEVEL is set (not set in
 *              this project); mtval does not exist in struct arch_esf
 *              on RISC-V Zephyr in any config. See
 *              esp32c3/zephyr/pir/src/trinity_fault_esp.c for the
 *              verified detail.
 *
 *          TRINITY_ARCH_XTENSA -- fully reserved. No board wired yet.
 *          hub/cam/doorbell's Xtensa builds (same shared
 *          trinity_panic_idf.c as motor, selected when
 *          CONFIG_IDF_TARGET_ARCH_RISCV is unset) currently set arch =
 *          TRINITY_ARCH_XTENSA with arch_data left zeroed -- that's a
 *          safe fallback for a shared file, not real Xtensa integration.
 *          Xtensa frame parsing (XtExcFrame or equivalent) is out of
 *          scope for this pass and hasn't been verified against either
 *          esp32 or esp32s3 IDF headers.
 ******************************************************************************/

#ifndef INCLUDE_TRINITY_CRASH_H_
#define INCLUDE_TRINITY_CRASH_H_

#include <stdint.h>

/* Uses TRINITY_ERROR_E -- defined in trinity_log.h's common section.
 * trinity_log.h includes this header after that enum, so it's always
 * available by the time this file is expanded from there. If included
 * standalone, include trinity_log.h first. */

#define TRINITY_CRASH_MAGIC      0x54524358u  /**< 'TRCX' -- record present */
#define TRINITY_CRASH_VERSION    2u

typedef enum
{
   TRINITY_ARCH_CORTEX_M = 0,  /**< F411, F103, nRF52840 (x4 boards)  */
   TRINITY_ARCH_XTENSA,        /**< reserved -- not yet wired        */
   TRINITY_ARCH_RISCV,         /**< ESP32-C3 (motor, pir)            */
} TRINITY_ARCH_E;

/**
 * \brief Canonical in-memory crash record. NOT the on-storage layout for
 *        any backend -- FRAM keeps its existing CRASH_LOG_ENTRY_X, RTC
 *        backup regs keep their existing 4-word layout, nRF flash log
 *        keeps its own text-line format. Each board's trinity_crash_store()
 *        picks which fields of this record it can actually persist to its
 *        own backend(s).
 */
typedef struct
{
   uint32_t         magic;          /**< TRINITY_CRASH_MAGIC if valid    */
   uint16_t         version;        /**< TRINITY_CRASH_VERSION           */
   uint16_t         arch;           /**< TRINITY_ARCH_E                  */

   TRINITY_ERROR_E  error;

   uint32_t         reset_reason;   /**< Board reset-cause flags (e.g.
                                      *   RCC CSR on STM32). Not currently
                                      *   captured by any board's fault
                                      *   handler -- reserved, set 0.     */

   uint32_t         pc;             /**< Faulting PC, 0 if unavailable   */
   uint32_t         lr;             /**< Link register at fault, 0 if
                                      *   unavailable                    */

   union
   {
      struct
      {
         uint32_t cfsr;   /**< SCB->CFSR -- populated on STM32 + nRF52840 */
         uint32_t hfsr;   /**< SCB->HFSR -- populated on nRF52840 only,
                            *   reserved on STM32                        */
         uint32_t dfsr;   /**< SCB->DFSR -- reserved, not yet captured  */
         uint32_t afsr;   /**< SCB->AFSR -- reserved, not yet captured  */
         uint32_t mmfar;  /**< SCB->MMFAR -- populated on nRF52840 only,
                            *   reserved on STM32                        */
         uint32_t bfar;   /**< SCB->BFAR -- populated on nRF52840 only,
                            *   reserved on STM32                        */
         uint32_t icsr;   /**< SCB->ICSR -- reserved, not yet captured  */
         uint32_t shcsr;  /**< SCB->SHCSR -- reserved, not yet captured */
      } cortex_m;

      struct
      {
         uint32_t exccause;  /**< reserved -- Xtensa not yet wired */
         uint32_t excvaddr;
         uint32_t ps;
         uint32_t epc1;
         uint32_t epc2;
      } xtensa;

      struct
      {
         uint32_t mcause;    /**< IDF: real. Zephyr: absent from struct
                               *   arch_esf here (see file \note above)  */
         uint32_t mepc;      /**< IDF + Zephyr: both real                */
         uint32_t mtval;     /**< IDF: real. Zephyr: doesn't exist in
                               *   struct arch_esf on RISC-V (any config) */
         uint32_t mstatus;   /**< IDF + Zephyr: both real                */
      } riscv;

   } arch_data;

} TRINITY_CRASH_RECORD_X;

/**
 * \brief Persist a crash record. Single entry point per board -- the
 *        implementation decides how/where, based on its own board's
 *        available backends and calling context (e.g. F411 checks
 *        whether it's running in exception context via IPSR to choose
 *        between its RTC and FRAM backends; F103 always uses FRAM, its
 *        only backend). See each board's trinity_fram_stm32_*.c,
 *        trinity_flash.c (nRF52840), or trinity_nvs_idf.c (ESP-IDF) for
 *        the actual decision.
 * \param p_record Record to persist. Caller owns the memory.
 */
void trinity_crash_store(TRINITY_CRASH_RECORD_X *p_record);

#endif /* INCLUDE_TRINITY_CRASH_H_ */
