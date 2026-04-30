/******************************************************************************
 * \file    wifi.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   WiFi initialisation and event group for ESP32-C3 motor controller.
 ******************************************************************************/

#ifndef WIFI_H
#define WIFI_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_CONNECTED_BIT   BIT0

/**< Event group shared with tcp_server and motor_control tasks.
 *   Set when IP is obtained. Tasks wait on WIFI_CONNECTED_BIT
 *   before opening sockets or sending data. */
extern EventGroupHandle_t g_wifi_eg;

/**
 * \brief  Initialise WiFi in STA mode with static IP.
 *         Registers event handlers, connects, sets WIFI_CONNECTED_BIT
 *         on IP acquisition.
 */
void wifi_init(void);

#endif /* WIFI_H */
