/******************************************************************************
 * \file aws_manager.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief AWS IoT Lambda manager interface for ESP32 hub node.
 *
 * \details Provides initialization and task entry point for sending
 *          consolidated sensor state to AWS Lambda every 5 minutes.
 *          See aws_manager.c for implementation details.
 ******************************************************************************/

#ifndef INCLUDE_AWS_MANAGER_H_
#define INCLUDE_AWS_MANAGER_H_

#include "freertos/event_groups.h"

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Initialize the AWS manager.
 *  \return void */
void aws_manager_init(void);

/** \brief AWS manager FreeRTOS task.
 *  \param p_wifi_eg - WiFi event group, waits for WIFI_CONNECTED_BIT.
 *  \param p_events  - Vroom bus event group for sensor events.
 *  \return void */
void aws_manager_task(EventGroupHandle_t p_wifi_eg,
                      EventGroupHandle_t p_events);

#endif /* INCLUDE_AWS_MANAGER_H_ */
