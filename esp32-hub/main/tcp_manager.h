/******************************************************************************
 * \file tcp_manager.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief TCP manager public interface for ESP32 hub node.
 *
 * \details Manages non-blocking TCP connections to BeagleBone and
 *          ESP32-C3 motor controller. Sends consolidated JSON sensor
 *          payload every TCP_SEND_INTERVAL_MS. See tcp_manager.c for
 *          implementation details.
 ******************************************************************************/

#ifndef INCLUDE_TCP_MANAGER_H_
#define INCLUDE_TCP_MANAGER_H_

#include "freertos/event_groups.h"

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Initialize the TCP manager.
 *  \return void */
void tcp_manager_init(void);

/** \brief TCP manager FreeRTOS task.
 *  \param p_system_eg - System event group handle.
 *  \param p_wifi_eg   - WiFi event group, waits for WIFI_CONNECTED_BIT.
 *  \param p_events    - Vroom bus event group for sensor events.
 *  \return void */
void tcp_manager_task(EventGroupHandle_t p_system_eg,
                      EventGroupHandle_t p_wifi_eg,
                      EventGroupHandle_t p_events);

#endif /* INCLUDE_TCP_MANAGER_H_ */
