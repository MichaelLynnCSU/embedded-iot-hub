/******************************************************************************
 * \file    tcp_client.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   TCP client API for ESP32-C3 motor node.
 *
 * \details Replaces tcp_server.h. No g_client_sock global -- connection is
 *          ephemeral per wake cycle.
 ******************************************************************************/

#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <stdint.h>

/**
 * \brief  Block until WiFi up, connect to hub, exchange one PWM/batt frame.
 *         Called from motor_task() on each wake from deep sleep.
 */
void     tcp_client_run_once(void);

/**
 * \brief  Connect, exchange, and close. Returns duty applied (0 on failure).
 *         Exposed for unit-test / direct call from motor_task if needed.
 */
uint32_t tcp_client_exchange(void);

#endif /* TCP_CLIENT_H */
