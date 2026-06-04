/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    log.c
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   USB-CDC log ring-buffer implementation.
 *
 * \details Implements a fixed-depth queue for log strings. log_enqueue()
 *          stores a message; log_drain() attempts to forward queued messages
 *          to the USB CDC interface when the host is connected.
 *
 * \note    RTT is written directly from log_enqueue() rather than being
 *          deferred through log_drain(). The previous design tracked RTT
 *          position with a static rtt_tail index that was compared against
 *          g_log_q_head. Because both indices are modulo LOG_QUEUE_DEPTH (6),
 *          they periodically land on the same value after the queue cycles,
 *          causing the drain loop to exit immediately and RTT to go silent
 *          indefinitely while the board continues running normally.
 *
 *          SEGGER_RTT_WriteNoLock() is a non-blocking RAM memcpy -- it
 *          returns immediately whether or not a host is connected -- so
 *          calling it inline from log_enqueue() adds no meaningful latency
 *          and avoids the index aliasing bug entirely. The USB CDC pipeline
 *          is unchanged and still uses the deferred queue in log_drain().
 ******************************************************************************/

#include "log.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>
#include "SEGGER_RTT.h"

/************************** STATIC (PRIVATE) DATA *****************************/

static char             g_log_queue[LOG_QUEUE_DEPTH][LOG_LINE_LEN]; /**< Ring buffer storage      */
static uint8_t          g_log_q_head  = 0u;                         /**< Write index              */
static uint8_t          g_log_q_tail  = 0u;                         /**< Read index               */
static volatile uint8_t g_log_q_count = 0u;                         /**< Number of pending lines  */

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  Enqueue a log message for deferred USB-CDC transmission and write
 *         it immediately to RTT.
 *
 * \param  p_msg - Null-terminated string to enqueue.
 *
 * \return void
 *
 * \warning Silently drops USB-CDC message when queue is full. RTT write is
 *          still attempted even when the USB queue is full.
 *          Not ISR-safe.
 *
 * \author MichaelLynnCSU
 */
void log_enqueue(const char *p_msg)
{
   if (NULL == p_msg)
   {
      return;
   }

   /* Write to RTT immediately.
    * SEGGER_RTT_WriteNoLock() is non-blocking: it copies into the RTT
    * up-buffer in RAM and returns, dropping bytes silently if the buffer
    * is full. No host connection is required. */
   (void)SEGGER_RTT_WriteNoLock(0u,
                                p_msg,
                                (unsigned int)strlen(p_msg));

   /* Enqueue for deferred USB-CDC transmission. */
   if (g_log_q_count >= LOG_QUEUE_DEPTH)
   {
      return;
   }

   (void)strncpy(g_log_queue[g_log_q_head], p_msg, LOG_LINE_LEN - 1u);
   g_log_queue[g_log_q_head][LOG_LINE_LEN - 1u] = '\0';
   g_log_q_head = (uint8_t)((g_log_q_head + 1u) % LOG_QUEUE_DEPTH);
   g_log_q_count++;
}

/**
 * \brief  Drain queued log messages to USB CDC when host is connected.
 *
 * \param  void
 *
 * \return void
 *
 * \details Stops draining on USBD_BUSY to prevent blocking the main loop.
 *          RTT is no longer drained here; see log_enqueue().
 *
 * \author MichaelLynnCSU
 */
void log_drain(void)
{
    extern USBD_HandleTypeDef hUsbDeviceFS;
    uint8_t ret = 0u;

    if (USBD_STATE_CONFIGURED != hUsbDeviceFS.dev_state)
    {
        return;
    }

    while (g_log_q_count > 0u)
    {
        ret = CDC_Transmit_FS((uint8_t *)g_log_queue[g_log_q_tail],
                              (uint16_t)strlen(g_log_queue[g_log_q_tail]));
        if (USBD_BUSY == ret)
        {
            break;
        }

        g_log_q_tail  = (uint8_t)((g_log_q_tail + 1u) % LOG_QUEUE_DEPTH);
        g_log_q_count--;
    }
}
