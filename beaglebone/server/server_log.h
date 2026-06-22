/******************************************************************************
 * \file server_log.h
 * \brief Log routing declarations for sensor server modules.
 *
 * \details Three writers, three destinations:
 *
 *   log_msg()        -> sensor_server.log
 *                       Operational lines: [SERVER], [UART], [PARSE], [PIPE].
 *                       All modules may call this.
 *
 *   log_telemetry()  -> telemetry.log
 *                       Exactly one call per frame from json_parser.c.
 *                       Numeric and boolean device state only — no strings,
 *                       no semantic decoding. High-volume; rotate aggressively.
 *
 *   log_event()      -> events.log
 *                       One call per events[] entry from hub delta gate.
 *                       Low-volume; every line is a true state transition.
 *                       Rotate slowly — this is the audit trail.
 *
 * File handles and implementations live in sensor_server.c.
 * seq is the join key across all three files.
 ******************************************************************************/

#ifndef SERVER_LOG_H
#define SERVER_LOG_H

void log_msg(const char *p_fmt, ...);
void log_telemetry(const char *p_fmt, ...);
void log_event(const char *p_fmt, ...);

#endif /* SERVER_LOG_H */
