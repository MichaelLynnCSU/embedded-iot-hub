#ifndef RING_BUFFER_H
#define RING_BUFFER_H

/******************************************************************************
 * Copyright (c) 2025 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ring_buffer.h
 * \author  MichaelLynnCSU
 * \date    01-01-2025
 *
 * \brief   Ring-buffer public interface — BluePill (STM32F103).
 *
 * \details Two independent sections:
 *
 *   SRAM ring buffer — generic byte FIFO used by existing callers.
 *
 *   FRAM crash log   — owns g_crash_write_ptr and g_total_crash_entries
 *                      (moved from fram_driver.c).  Provides:
 *                        rb_crashlog_init()     restore state from FRAM meta
 *                        rb_crashlog_push()     write one crash entry
 *                        rb_crashlog_get_total() query cumulative count
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* SRAM RING BUFFER                                                          */
/*===========================================================================*/

/** Opaque ring-buffer control block. */
typedef struct
{
   uint8_t  *p_buf;    /**< Backing store         */
   uint16_t  capacity; /**< Buffer size in bytes  */
   uint16_t  head;     /**< Read index            */
   uint16_t  tail;     /**< Write index           */
   uint16_t  count;    /**< Bytes currently held  */
} RingBuffer_X;

/**
 * \brief  Initialise a ring buffer control block.
 *
 * \param  p_rb       - Pointer to control block to initialise.
 * \param  p_storage  - Backing byte array.
 * \param  size       - Size of backing array in bytes.
 *
 * \return void
 */
void     rb_init(RingBuffer_X *p_rb, uint8_t *p_storage, uint16_t size);

/**
 * \brief  Push one byte into the ring buffer (overwrites oldest on full).
 *
 * \param  p_rb  - Pointer to control block.
 * \param  byte  - Byte to push.
 *
 * \return void
 */
void     rb_push(RingBuffer_X *p_rb, uint8_t byte);

/**
 * \brief  Pop one byte from the ring buffer.
 *
 * \param  p_rb    - Pointer to control block.
 * \param  p_byte  - Out: byte read.
 *
 * \return bool - true if a byte was available, false if buffer was empty.
 */
bool     rb_pop(RingBuffer_X *p_rb, uint8_t *p_byte);

/**
 * \brief  Return number of bytes currently in the ring buffer.
 *
 * \param  p_rb - Pointer to control block.
 *
 * \return uint16_t - Byte count.
 */
uint16_t rb_count(const RingBuffer_X *p_rb);

/**
 * \brief  Return true if the ring buffer is empty.
 *
 * \param  p_rb - Pointer to control block.
 *
 * \return bool
 */
bool     rb_is_empty(const RingBuffer_X *p_rb);

/*===========================================================================*/
/* FRAM CRASH LOG                                                             */
/*===========================================================================*/

/**
 * \brief  Restore crash-log state from FRAM metadata.
 *
 * \details Must be called once after FRAM_Init() and after FRAM availability
 *          has been confirmed.  Reads both crash fields (write pointer and
 *          total count) from the FRAM meta block via FRAM_LoadMeta().
 *          Also restores the temp-log fields so a single FRAM read serves
 *          the whole system.
 *
 * \return void
 */
void rb_crashlog_init(void);

/**
 * \brief  Write one crash entry to the FRAM crash partition.
 *
 * \details Replaces FRAM_LogCrash().  Computes CRC-8, writes the entry at
 *          the current ring pointer, advances the pointer, increments the
 *          total count, and persists all metadata via FRAM_SaveMeta().
 *
 * \param  error_code  - TRINITY_ERROR_E value cast to uint8_t.
 * \param  boot_count  - Current boot counter value.
 *
 * \return void
 */
void rb_crashlog_push(uint8_t error_code, uint8_t boot_count);

/**
 * \brief  Return cumulative crash entry count.
 *
 * \details Replaces FRAM_GetTotalCrashes().
 *
 * \return uint32_t - Number of crash entries ever written.
 */
uint32_t rb_crashlog_get_total(void);

#endif /* RING_BUFFER_H */
