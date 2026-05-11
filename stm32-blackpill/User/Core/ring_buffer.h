/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ring_buffer.h
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   Ring buffer public interface — STM32F411 BlackPill.
 *
 * \details Exposes two independent ring buffers:
 *
 *   1. SRAM — UART line queue.
 *      Fixed-depth queue for assembling UART bytes into complete lines
 *      and dispatching them to the main loop without blocking the ISR.
 *      Enqueue (rb_push_byte) is ISR-safe. Dequeue (rb_dequeue) must be
 *      called from main-loop context only.
 *
 *   2. FRAM — Crash log queue (MB85RC256V over I2C).
 *      Persistent ring buffer for CRASH_LOG_ENTRY_X records. State
 *      (write pointer, entry count) survives power loss via the FRAM meta
 *      block. Call rb_crashlog_init() once at boot after fram_init().
 ******************************************************************************/

#ifndef INCLUDE_RING_BUFFER_H_
#define INCLUDE_RING_BUFFER_H_

#include <stdint.h>

/******************************** CONSTANTS ***********************************/

#define UART_QUEUE_DEPTH  8u    /**< Maximum number of pending UART lines     */
#define UART_LINE_LEN     128u  /**< Maximum characters per line (incl NUL)   */

/*===========================================================================*
 * RING BUFFER 1 — SRAM  — UART line queue
 *===========================================================================*/

/**
 * \brief  Initialise the UART ring buffer to a known empty state.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void rb_init(void);

/**
 * \brief  Feed one received byte into the line-assembly buffer.
 *
 * \param  byte - Raw received byte from the UART ISR.
 *
 * \return void
 *
 * \details CR or LF terminates a line and enqueues it. Overflow bytes are
 *          discarded and the assembly index reset. ISR-safe.
 *
 * \author MichaelLynnCSU
 */
void rb_push_byte(uint8_t byte);

/**
 * \brief  Dequeue one complete UART line into the caller's buffer.
 *
 * \param  p_dst - Destination buffer of at least UART_LINE_LEN bytes.
 *
 * \return uint8_t - 1 if a line was dequeued, 0 if the queue was empty.
 *
 * \warning Must be called from main-loop context only (not ISR).
 *
 * \author MichaelLynnCSU
 */
uint8_t rb_dequeue(char *p_dst);

/**
 * \brief  Return the number of complete lines currently queued.
 *
 * \param  void
 *
 * \return uint8_t - Pending line count (0 .. UART_QUEUE_DEPTH).
 *
 * \author MichaelLynnCSU
 */
uint8_t rb_count(void);

/*===========================================================================*
 * RING BUFFER 2 — FRAM  — Crash log (MB85RC256V over I2C)
 *===========================================================================*/

/**
 * \brief  Restore crash log state (write pointer, entry count) from FRAM.
 *
 * \param  void
 *
 * \return void
 *
 * \note   Must be called once at boot after fram_init(). Also call after
 *         erasing the crash log to reset local state.
 *
 * \author MichaelLynnCSU
 */
void rb_crashlog_init(void);

/**
 * \brief  Enqueue a trinity crash event into the FRAM crash log partition.
 *
 * \param  error_code - TRINITY_ERROR_E value cast to uint8_t.
 * \param  boot_count - Current boot counter value.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void rb_crashlog_push(uint8_t error_code, uint8_t boot_count);

/**
 * \brief  Return the cumulative number of crash entries ever logged.
 *
 * \param  void
 *
 * \return uint32_t - Total crash entry count (across all boots).
 *
 * \author MichaelLynnCSU
 */
uint32_t rb_crashlog_get_total(void);

#endif /* INCLUDE_RING_BUFFER_H_ */
