/******************************************************************************
 * \file    trinity_crash.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief   Unified multi-architecture crash record contract -- all Trinity
 *          boards (STM32 today; Xtensa/RISC-V arch_data reserved for when
 *          an ESP32/RISC-V board is actually wired to this contract -- see
 *          \note below).
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
 * \note    SCOPE: only TRINITY_ARCH_CORTEX_M is populated today (F411,
 *          F103). TRINITY_ARCH_XTENSA/RISCV and their arch_data members
 *          exist so the contract doesn't need another breaking revision
 *          when an ESP32 (Xtensa/RISC-V) or other RISC-V board adopts it,
 *          but nothing currently writes or reads them -- this is
 *          deliberately ahead of any real Xtensa/RISC-V wiring, which is
 *          separate, not-yet-scheduled work.
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
   TRINITY_ARCH_CORTEX_M = 0,  /**< F411, F103 (both boards today)  */
   TRINITY_ARCH_XTENSA,        /**< reserved -- not yet wired        */
   TRINITY_ARCH_RISCV,         /**< reserved -- not yet wired        */
} TRINITY_ARCH_E;

/**
 * \brief Canonical in-memory crash record. NOT the on-storage layout for
 *        any backend -- FRAM keeps its existing CRASH_LOG_ENTRY_X, RTC
 *        backup regs keep their existing 4-word layout. Each board's
 *        trinity_crash_store() picks which fields of this record it can
 *        actually persist to its own backend(s).
 */
typedef struct
{
   uint32_t         magic;          /**< TRINITY_CRASH_MAGIC if valid    */
   uint16_t         version;        /**< TRINITY_CRASH_VERSION           */
   uint16_t         arch;           /**< TRINITY_ARCH_E                  */

   TRINITY_ERROR_E  error;

   uint32_t         reset_reason;   /**< Board reset-cause flags (e.g.
                                      *   RCC CSR on STM32). Not currently
                                      *   captured by either board's fault
                                      *   handler -- reserved, set 0.     */

   uint32_t         pc;             /**< Faulting PC, 0 if unavailable   */
   uint32_t         lr;             /**< Link register at fault, 0 if
                                      *   unavailable                    */

   union
   {
      struct
      {
         uint32_t cfsr;   /**< SCB->CFSR -- populated on Cortex-M       */
         uint32_t hfsr;   /**< SCB->HFSR -- reserved, not yet captured  */
         uint32_t dfsr;   /**< SCB->DFSR -- reserved, not yet captured  */
         uint32_t afsr;   /**< SCB->AFSR -- reserved, not yet captured  */
         uint32_t mmfar;  /**< SCB->MMFAR -- reserved, not yet captured */
         uint32_t bfar;   /**< SCB->BFAR -- reserved, not yet captured  */
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
         uint32_t mcause;    /**< reserved -- RISC-V not yet wired */
         uint32_t mepc;
         uint32_t mtval;
         uint32_t mstatus;
      } riscv;

   } arch_data;

} TRINITY_CRASH_RECORD_X;

/**
 * \brief Persist a crash record. Single entry point per board -- the
 *        implementation decides how/where, based on its own board's
 *        available backends and calling context (e.g. F411 checks
 *        whether it's running in exception context via IPSR to choose
 *        between its RTC and FRAM backends; F103 always uses FRAM, its
 *        only backend). See each board's trinity_fram_stm32_*.c for the
 *        actual decision.
 * \param p_record Record to persist. Caller owns the memory.
 */
void trinity_crash_store(TRINITY_CRASH_RECORD_X *p_record);

#endif /* INCLUDE_TRINITY_CRASH_H_ */
