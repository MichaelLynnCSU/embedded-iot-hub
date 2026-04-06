/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    log.h
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   USB-CDC log ring-buffer public interface.
 *
 * \details Provides a lightweight log queue for deferred transmission over
 *          USB CDC. Safe to enqueue from main-loop context; Log_Drain()
 *          must be called periodically from the main loop only.
 ******************************************************************************/

#ifndef INCLUDE_LOG_H_
#define INCLUDE_LOG_H_

/******************************** CONSTANTS ***********************************/

#define LOG_QUEUE_DEPTH  6u   /**< Maximum number of pending log lines       */
#define LOG_LINE_LEN    64u   /**< Maximum characters per log line (incl NUL)*/

/*************************** FUNCTION PROTOTYPES ******************************/

/**
 * \brief  Enqueue a log message for deferred USB-CDC transmission.
 *
 * \param  p_msg - Null-terminated string to enqueue. Truncated to
 *                 LOG_LINE_LEN - 1 characters if longer.
 *
 * \return void
 *
 * \warning Not safe to call from ISR context. Drops message silently when
 *          queue is full.
 *
 * \author MichaelLynnCSU
 */
void log_enqueue(const char *p_msg);

/**
 * \brief  Drain queued log messages to USB CDC when host is connected.
 *
 * \param  void
 *
 * \return void
 *
 * \details Must be called repeatedly from the main loop. Stops draining on
 *          USBD_BUSY to avoid blocking.
 *
 * \author MichaelLynnCSU
 */
void log_drain(void);

/******************************** COMPATIBILITY *******************************/

/* TODO: Remove once crash_log.c is updated to call log_enqueue() directly. */
#define Log(msg)  log_enqueue(msg)

#endif /* INCLUDE_LOG_H_ */
