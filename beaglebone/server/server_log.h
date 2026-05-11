/******************************************************************************
 * \file server_log.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-05-10
 *
 * \brief Shared log interface for sensor server modules.
 *
 * \details Extracted from sensor_server.c (2026-05-10) so that
 *          uart_io.c, pipe_writer.c, and json_parser.c can all call
 *          log_msg() without depending on sensor_server.c directly.
 *          sensor_server.c owns the log_fp handle and calls log_init().
 ******************************************************************************/

#ifndef SERVER_LOG_H
#define SERVER_LOG_H

/******************************************************************************
 * \brief Write timestamped message to stdout and log file.
 *
 * \param p_fmt - printf-style format string.
 * \param ...   - Format arguments.
 ******************************************************************************/
void log_msg(const char *p_fmt, ...);

/******************************************************************************
 * \brief Initialize log file with line buffering.
 *
 * \return void
 ******************************************************************************/
void log_init(void);

#endif /* SERVER_LOG_H */
