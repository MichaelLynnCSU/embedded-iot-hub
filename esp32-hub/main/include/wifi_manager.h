/******************************************************************************
 * \file wifi_manager.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief WiFi manager public interface for ESP32 hub node.
 *
 * \details Manages WiFi STA connection with exponential backoff retry.
 *          See wifi_manager.c for implementation details.
 ******************************************************************************/

#ifndef INCLUDE_WIFI_MANAGER_H_
#define INCLUDE_WIFI_MANAGER_H_

#include "freertos/event_groups.h"

/******************************** CONSTANTS ***********************************/

#define WIFI_CONNECTED_BIT  BIT0  /**< set in wifi_event_group on IP acquired */

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Initialize the WiFi manager.
 *  \return void */
void wifi_manager_init(void);

/** \brief WiFi manager FreeRTOS task.
 *  \param p_system_eg - System event group handle.
 *  \param p_wifi_eg   - WiFi event group, sets WIFI_CONNECTED_BIT on connect.
 *  \return void */
void wifi_manager_task(EventGroupHandle_t p_system_eg,
                       EventGroupHandle_t p_wifi_eg);

#endif /* INCLUDE_WIFI_MANAGER_H_ */
