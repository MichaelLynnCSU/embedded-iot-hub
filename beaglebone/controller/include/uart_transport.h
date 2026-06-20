/******************************************************************************
 * \file uart_transport.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-15
 *
 * \brief UART transport layer for BeagleBone data controller.
 *
 * \details Extracted from uart_controller.c as part of the
 *          transport/protocol/correlation/lock-adapter split (no behavior
 *          change). This module is pure I/O + buffering only — it has no
 *          knowledge of doorbell, lock, or protocol meaning.
 *
 *          Owns:
 *            - UART fd lifecycle (uart_open, uart_reader_thread)
 *            - line framing + dispatch (uart_parse_line, internal)
 *            - ring buffer push/pop (declared in uart_controller.h for
 *              backward-compat linkage; implemented here)
 *            - uart_push_msg() — the only UART write primitive, exported
 *              for uart_protocol.c
 *
 *          uart_reader_thread(), uart_ring_push(), uart_ring_pop(), and
 *          uart_sem/ring externs remain declared in uart_controller.h
 *          (public controller API, unchanged). This header adds only the
 *          new internal export needed by uart_protocol.c.
 ******************************************************************************/
#ifndef INCLUDE_UART_TRANSPORT_H_
#define INCLUDE_UART_TRANSPORT_H_
#include "config.h"

/** \brief UART outbound message envelope. */
typedef struct
{
   char     buf[UART_MSG_BUF_SIZE];
   int      len;
   uint64_t event_id;
   int      has_event_id;
} UartMsg;

/******************************************************************************
 * \brief Write a UartMsg envelope to UART with mutex protection.
 *
 * \param p_msg - Pointer to UartMsg envelope.
 *
 * \return void
 *
 * \details The only UART write primitive. Exported for uart_controller.c
 *          (build_and_push() / uart_update_frame()) and doorbell_pending.c.
 ******************************************************************************/
void uart_push_msg(const UartMsg *p_msg);

/******************************************************************************
 * \brief Return 1 if the UART device is currently open, 0 otherwise.
 *
 * \return int - 1 if open, 0 if not.
 *
 * \details Exported for uart_protocol.c, which previously read g_uart_fd
 *          directly (g_uart_fd < 0 check at top of uart_update_frame() and
 *          uart_push_thread()'s "if (0 > g_uart_fd) continue;" guard).
 *          g_uart_fd itself remains private to uart_transport.c.
 ******************************************************************************/
int uart_transport_is_open(void);

#endif /* INCLUDE_UART_TRANSPORT_H_ */
