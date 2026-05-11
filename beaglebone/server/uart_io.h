/******************************************************************************
 * \file uart_io.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-05-10
 *
 * \brief UART transport interface for sensor server.
 *
 * \details Owns UART init and the double buffer accumulation loop.
 *          Knows nothing about JSON parsing or the controller pipe.
 *          Split from sensor_server.c (2026-05-10) -- see sensor_server.c
 *          file header for full rationale.
 ******************************************************************************/

#ifndef UART_IO_H
#define UART_IO_H

#include <stdbool.h>

/******************************************************************************
 * \brief Initialize UART at 115200 8N1.
 *
 * \return int - 0 on success, -1 on failure.
 ******************************************************************************/
int uart_io_init(void);

/******************************************************************************
 * \brief Close UART file descriptor.
 *
 * \return void
 ******************************************************************************/
void uart_io_close(void);

/******************************************************************************
 * \brief Read available bytes from UART into the active accumulation buffer.
 *
 * \return void
 *
 * \details Caller should call uart_io_next_frame() after each call to
 *          drain all complete frames before reading again.
 ******************************************************************************/
void uart_io_read(void);

/******************************************************************************
 * \brief Extract the next complete JSON frame from the active buffer.
 *
 * \param p_out  - Output buffer to copy frame into.
 * \param out_sz - Size of output buffer.
 *
 * \return bool - true if a complete frame was found and copied.
 *
 * \details Copies frame into p_out (stable copy), then compacts the
 *          active buffer so remaining bytes are preserved for the next
 *          call. Call in a loop until false to drain back-to-back frames.
 ******************************************************************************/
bool uart_io_next_frame(char *p_out, int out_sz);

#endif /* UART_IO_H */
