/******************************************************************************
 * Copyright (c) 2025 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ring_buffer.c
 * \author  MichaelLynnCSU
 * \date    01-01-2025
 *
 * \brief   Ring-buffer implementation — BluePill (STM32F103).
 *
 * \details Two independent sections:
 *
 *   SRAM ring buffer — generic byte FIFO.
 *
 *   FRAM crash log   — owns g_crash_write_ptr and g_total_crash_entries,
 *                      previously held in fram_driver.c.
 *                      calc_crc8() is duplicated here (static) to keep the
 *                      two drivers independent with no shared helper header.
 ******************************************************************************/

#include "ring_buffer.h"
#include "fram_driver.h"
#include <string.h>

/*===========================================================================*/
/* SRAM RING BUFFER                                                          */
/*===========================================================================*/

/**
 * \brief  Initialise a ring buffer control block.
 *
 * \param  p_rb      - Pointer to control block.
 * \param  p_storage - Backing byte array.
 * \param  size      - Size of backing array in bytes.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void rb_init(RingBuffer_X *p_rb, uint8_t *p_storage, uint16_t size)
{
   if ((NULL == p_rb) || (NULL == p_storage) || (0u == size)) { return; }

   p_rb->p_buf    = p_storage;
   p_rb->capacity = size;
   p_rb->head     = 0u;
   p_rb->tail     = 0u;
   p_rb->count    = 0u;
}

/**
 * \brief  Push one byte; overwrites oldest byte when full.
 *
 * \param  p_rb  - Pointer to control block.
 * \param  byte  - Byte to push.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void rb_push(RingBuffer_X *p_rb, uint8_t byte)
{
   if (NULL == p_rb) { return; }

   p_rb->p_buf[p_rb->tail] = byte;
   p_rb->tail = (uint16_t)((p_rb->tail + 1u) % p_rb->capacity);

   if (p_rb->count < p_rb->capacity)
   {
      p_rb->count++;
   }
   else
   {
      /* Overwrite: advance head to discard oldest byte */
      p_rb->head = (uint16_t)((p_rb->head + 1u) % p_rb->capacity);
   }
}

/**
 * \brief  Pop one byte from the ring buffer.
 *
 * \param  p_rb   - Pointer to control block.
 * \param  p_byte - Out: byte read.
 *
 * \return bool - true if a byte was available.
 *
 * \author MichaelLynnCSU
 */
bool rb_pop(RingBuffer_X *p_rb, uint8_t *p_byte)
{
   if ((NULL == p_rb) || (NULL == p_byte)) { return false; }
   if (0u == p_rb->count)                  { return false; }

   *p_byte    = p_rb->p_buf[p_rb->head];
   p_rb->head = (uint16_t)((p_rb->head + 1u) % p_rb->capacity);
   p_rb->count--;

   return true;
}

/**
 * \brief  Return number of bytes currently in the ring buffer.
 *
 * \param  p_rb - Pointer to control block.
 *
 * \return uint16_t
 *
 * \author MichaelLynnCSU
 */
uint16_t rb_count(const RingBuffer_X *p_rb)
{
   if (NULL == p_rb) { return 0u; }
   return p_rb->count;
}

/**
 * \brief  Return true if the ring buffer is empty.
 *
 * \param  p_rb - Pointer to control block.
 *
 * \return bool
 *
 * \author MichaelLynnCSU
 */
bool rb_is_empty(const RingBuffer_X *p_rb)
{
   if (NULL == p_rb) { return true; }
   return (0u == p_rb->count);
}

/*===========================================================================*/
/* FRAM CRASH LOG — private state                                            */
/*===========================================================================*/

/** Write offset within the FRAM crash partition (byte index). */
static uint16_t g_crash_write_ptr    = 0u;

/** Cumulative number of crash entries ever written. */
static uint32_t g_total_crash_entries = 0ul;

/**
 * \brief  Calculate CRC-8 over a data buffer.
 *
 * \details Private to this translation unit.  Identical polynomial to the
 *          copy previously in fram_driver.c (0x07, init 0xFF).
 *
 * \param  p_data - Pointer to data bytes.
 * \param  len    - Number of bytes to process.
 *
 * \return uint8_t - Computed CRC-8 value.
 *
 * \author MichaelLynnCSU
 */
static uint8_t calc_crc8(uint8_t *p_data, uint16_t len)
{
   uint8_t  crc = 0xFFu; /**< CRC accumulator */
   uint16_t i   = 0u;    /**< Byte index      */
   uint8_t  j   = 0u;    /**< Bit index       */

   for (i = 0u; i < len; i++)
   {
      crc ^= p_data[i];
      for (j = 0u; j < 8u; j++)
      {
         if (0u != (crc & 0x80u))
         {
            crc = (uint8_t)((crc << 1u) ^ 0x07u);
         }
         else
         {
            crc <<= 1u;
         }
      }
   }

   return crc;
}

/*===========================================================================*/
/* FRAM CRASH LOG — public API                                               */
/*===========================================================================*/

/**
 * \brief  Restore crash-log state from FRAM metadata.
 *
 * \details Calls FRAM_LoadMeta() with NULL for the temp out-params — we
 *          only need the crash fields here.  The temp-log state is managed
 *          internally by FRAM_LogTemp() via its own static variables.
 *
 *          Must be called once after FRAM_Init() and after FRAM availability
 *          has been confirmed (g_fram_ok == true in trinity_fram_stm32_f1.c).
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void rb_crashlog_init(void)
{
   FRAM_LoadMeta(NULL,
                 NULL,
                 &g_crash_write_ptr,
                 &g_total_crash_entries);
}

/**
 * \brief  Write one crash entry to the FRAM crash partition.
 *
 * \details Builds a CRASH_LOG_ENTRY_X, computes CRC-8, writes to FRAM at
 *          the current ring pointer, advances the pointer with wrap-around,
 *          increments the total, then persists all metadata via
 *          FRAM_SaveMeta().  The temp fields passed to FRAM_SaveMeta() are
 *          obtained from FRAM_GetWritePtr() / FRAM_GetTotalEntries() so the
 *          single meta block stays coherent.
 *
 * \param  error_code  - TRINITY_ERROR_E value cast to uint8_t.
 * \param  boot_count  - Current boot counter value.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void rb_crashlog_push(uint8_t error_code, uint8_t boot_count)
{
   CRASH_LOG_ENTRY_X entry   = {0}; /**< Crash entry to write */
   uint16_t          addr    = 0u;  /**< Computed write addr  */
   uint16_t          entry_sz = 0u; /**< Sizeof entry         */

   entry_sz = (uint16_t)sizeof(CRASH_LOG_ENTRY_X);

   entry.timestamp  = HAL_GetTick();
   entry.error_code = error_code;
   entry.boot_count = boot_count;
   entry.crc        = calc_crc8((uint8_t *)&entry,
                                 (uint16_t)(entry_sz - 1u));

   addr = (uint16_t)(FRAM_CRASHLOG_ADDR + g_crash_write_ptr);

   if (HAL_OK == FRAM_Write(addr, (uint8_t *)&entry, entry_sz))
   {
      g_crash_write_ptr += entry_sz;

      if ((g_crash_write_ptr + entry_sz) >= FRAM_CRASHLOG_SIZE)
      {
         g_crash_write_ptr = 0u;
      }

      g_total_crash_entries++;

      FRAM_SaveMeta(FRAM_GetWritePtr(),
                    FRAM_GetTotalEntries(),
                    g_crash_write_ptr,
                    g_total_crash_entries);
   }
}

/**
 * \brief  Return cumulative crash entry count.
 *
 * \return uint32_t
 *
 * \author MichaelLynnCSU
 */
uint32_t rb_crashlog_get_total(void)
{
   return g_total_crash_entries;
}
