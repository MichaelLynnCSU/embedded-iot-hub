/******************************************************************************
 * \file    tcp_server.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   TCP server task and shared client socket for ESP32-C3 motor node.
 ******************************************************************************/

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <stdbool.h>

/**< Shared client socket. Written by tcp_rx_task, read by motor_task.
 *   Check g_client_sock_valid before use. send() is safe from multiple
 *   tasks on lwIP. */
extern volatile int  g_client_sock;
extern volatile bool g_client_sock_valid;

/**
 * \brief  TCP server task. Listens on TCP_PORT, accepts one client at a time,
 *         parses incoming JSON, exposes socket for motor_task battery send-back.
 *         Never returns -- run via xTaskCreate().
 */
void tcp_rx_task(void *p_arg);

#endif /* TCP_SERVER_H */
