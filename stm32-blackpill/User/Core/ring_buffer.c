/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ring_buffer.c
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   Ring buffer implementations — STM32F411 BlackPill.
 *
 * \details Two independent ring buffers:
 *
 *   1. SRAM  — UART line queue (ISR → main loop, no persistence)
 *   2. FRAM  — Crash log queue (MB85RC256V over I2C, survives power loss)
 *
 *          The SRAM buffer assembles raw UART bytes into CR/LF-delimited
 *          lines and stores them in a fixed-depth circular queue. The main
 *          loop drains it with rb_dequeue() without blocking the ISR.
 *
 *          The FRAM buffer enqueues CRASH_LOG_ENTRY_X records into the
 *          FRAM crash partition. State (write pointer, entry count) is
 *          persisted to the FRAM meta block after every write. Call
 *          rb_crashlog_init() once at boot (after fram_init()) to restore
 *          state from FRAM.
 ******************************************************************************/

#include "ring_buffer.h"
#include "fram_driver.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/*===========================================================================*
 * RING BUFFER 1 — SRAM  — UART line queue
 *===========================================================================*/

/************************** STATIC (PRIVATE) DATA *****************************/

static char             g_uart_line[UART_LINE_LEN]         = {0}; /**< Assembly buffer               */
static uint8_t          g_uart_line_idx                    = 0u;  /**< Write position in assembly buf */
static char             g_queue[UART_QUEUE_DEPTH][UART_LINE_LEN]; /**< Completed line queue           */
static uint8_t          g_q_head                           = 0u;  /**< Queue write index              */
static uint8_t          g_q_tail                           = 0u;  /**< Queue read index               */
static volatile uint8_t g_q_count                          = 0u;  /**< Number of pending lines        */

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  Initialise the UART ring buffer to a known empty state.
 */
void rb_init(void)
{
   g_uart_line_idx = 0u;
   g_q_head        = 0u;
   g_q_tail        = 0u;
   g_q_count       = 0u;
   (void)memset(g_uart_line, 0, sizeof(g_uart_line));
   (void)memset(g_queue,     0, sizeof(g_queue));
}

/**
 * \brief  Feed one received byte into the line-assembly buffer.
 *
 * \param  byte - Raw received byte from the UART ISR.
 */
void rb_push_byte(uint8_t byte)
{
   char c = (char)byte;

   if (('\n' == c) || ('\r' == c))
   {
      if ((g_uart_line_idx > 0u) && (g_q_count < UART_QUEUE_DEPTH))
      {
         g_uart_line[g_uart_line_idx] = '\0';
         (void)memcpy(g_queue[g_q_head], g_uart_line, g_uart_line_idx + 1u);
         g_q_head = (uint8_t)((g_q_head + 1u) % UART_QUEUE_DEPTH);

         __disable_irq();
         g_q_count++;
         __enable_irq();
      }

      g_uart_line_idx = 0u;
   }
   else if (g_uart_line_idx < (uint8_t)(UART_LINE_LEN - 1u))
   {
      g_uart_line[g_uart_line_idx] = c;
      g_uart_line_idx++;
   }
   else
   {
      /* Line overflow — discard and reset */
      g_uart_line_idx = 0u;
   }
}

/**
 * \brief  Dequeue one complete UART line into the caller's buffer.
 *
 * \param  p_dst - Destination buffer of at least UART_LINE_LEN bytes.
 *
 * \return uint8_t - 1 if a line was dequeued, 0 if the queue was empty.
 */
uint8_t rb_dequeue(char *p_dst)
{
   if ((NULL == p_dst) || (0u == g_q_count))
   {
      return 0u;
   }

   (void)memcpy(p_dst, g_queue[g_q_tail], UART_LINE_LEN);
   g_q_tail = (uint8_t)((g_q_tail + 1u) % UART_QUEUE_DEPTH);

   __disable_irq();
   g_q_count--;
   __enable_irq();

   return 1u;
}

/**
 * \brief  Return the number of complete lines currently queued.
 */
uint8_t rb_count(void)
{
   return g_q_count;
}

/*===========================================================================*
 * RING BUFFER 2 — FRAM  — Crash log (MB85RC256V over I2C)
 *===========================================================================*/

/************************** SHARED UTILITIES **********************************/

/**
 * \brief  Calculate CRC-8 over a data buffer.
 *
 * \param  p_data - Pointer to data bytes.
 * \param  len    - Number of bytes to process.
 *
 * \return uint8_t - Computed CRC-8 value.
 */
static uint8_t calc_crc8(uint8_t *p_data, uint16_t len)
{
   uint8_t  crc = 0xFFu; /**< CRC accumulator, initialised to 0xFF */
   uint16_t i   = 0u;    /**< Byte index                           */
   uint8_t  j   = 0u;    /**< Bit index                            */

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

/************************** STATIC (PRIVATE) DATA *****************************/

static uint16_t g_crash_write_ptr     = 0u;  /**< Crash log write offset (FRAM partition) */
static uint32_t g_total_crash_entries = 0ul; /**< Cumulative crash entry count            */

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  Restore crash log state from FRAM meta block.
 *
 * \param  void
 *
 * \return void
 *
 * \note   Must be called once at boot after fram_init(). Also call after
 *         erasing the crash log (trinity_log_erase) to reset local state.
 *
 * \author MichaelLynnCSU
 */
void rb_crashlog_init(void)
{
   fram_load_meta(&g_crash_write_ptr, &g_total_crash_entries);
}

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
void rb_crashlog_push(uint8_t error_code, uint8_t boot_count)
{
   CRASH_LOG_ENTRY_X entry = {0}; /**< Crash entry to write */
   uint16_t          addr  = 0u;  /**< Computed write addr  */

   entry.timestamp  = HAL_GetTick();
   entry.error_code = error_code;
   entry.boot_count = boot_count;
   entry.crc        = calc_crc8((uint8_t *)&entry,
                                 (uint16_t)(sizeof(entry) - 1u));

   addr = (uint16_t)(FRAM_CRASHLOG_ADDR + g_crash_write_ptr);

   if (HAL_OK == fram_write(addr, (uint8_t *)&entry, (uint16_t)sizeof(entry)))
   {
      g_crash_write_ptr += (uint16_t)sizeof(entry);

      if ((g_crash_write_ptr + (uint16_t)sizeof(entry)) >= FRAM_CRASHLOG_SIZE)
      {
         g_crash_write_ptr = 0u; /* Wrap — ring is full */
      }
      else
      {
         /* Still space in crash partition */
      }

      g_total_crash_entries++;
      fram_save_meta(g_crash_write_ptr, g_total_crash_entries);
   }
   else
   {
      /* Write failed — state unchanged, meta not updated */
   }
}

/**
 * \brief  Return the cumulative number of crash entries ever logged.
 *
 * \param  void
 *
 * \return uint32_t - Total crash entry count (across all boots).
 *
 * \author MichaelLynnCSU
 */
uint32_t rb_crashlog_get_total(void)
{
   return g_total_crash_entries;
}
