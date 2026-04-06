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
 ******************************************************************************/

#include "log.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>

/************************** STATIC (PRIVATE) DATA *****************************/

static char             g_log_queue[LOG_QUEUE_DEPTH][LOG_LINE_LEN]; /**< Ring buffer storage      */
static uint8_t          g_log_q_head  = 0u;                         /**< Write index              */
static uint8_t          g_log_q_tail  = 0u;                         /**< Read index               */
static volatile uint8_t g_log_q_count = 0u;                         /**< Number of pending lines  */

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  Enqueue a log message for deferred USB-CDC transmission.
 *
 * \param  p_msg - Null-terminated string to enqueue.
 *
 * \return void
 *
 * \warning Silently drops message when queue is full. Not ISR-safe.
 *
 * \author MichaelLynnCSU
 */
void log_enqueue(const char *p_msg)
{
   if (NULL == p_msg)
   {
      return;
   }

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
 *
 * \author MichaelLynnCSU
 */
void log_drain(void)
{
   extern USBD_HandleTypeDef hUsbDeviceFS;
   uint8_t ret = 0u; /**< CDC transmit return code */

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
