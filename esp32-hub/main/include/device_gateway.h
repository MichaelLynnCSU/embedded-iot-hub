/******************************************************************************
 * \file device_gateway.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief AWS IoT Lambda manager interface for ESP32 hub node.
 *
 * \details Provides initialization and task entry point for sending
 *          consolidated sensor state to AWS Lambda every 5 minutes.
 *          Parses the Lambda response for PI controller parameters
 *          (kp, ki, kd, setpoint) and exposes them via getter functions
 *          for consumption by tcp_manager.c.
 *          See device_gateway.c for implementation details.
 ******************************************************************************/

#ifndef INCLUDE_AWS_MANAGER_H_
#define INCLUDE_AWS_MANAGER_H_

#include "freertos/event_groups.h"
#include "vroom_bus.h"
/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Initialize the AWS manager.
 *  \return void */
void device_gateway_init(void);

/** \brief AWS manager FreeRTOS task.
 *  \param p_wifi_eg - WiFi event group, waits for WIFI_CONNECTED_BIT.
 *  \param p_events  - Vroom bus event group for sensor events.
 *  \return void */
void device_gateway_task(EventGroupHandle_t p_wifi_eg,
                      BUS_SUBSCRIBER_T   sub);

/** \brief Get the PI controller proportional gain from last Lambda response.
 *  \return kp as float */
float gateway_get_kp(void);

/** \brief Get the PI controller integral gain from last Lambda response.
 *  \return ki as float */
float gateway_get_ki(void);

/** \brief Get the PI controller derivative gain from last Lambda response.
 *  \return kd as float */
float gateway_get_kd(void);

/** \brief Get the temperature setpoint from last Lambda response.
 *  \return setpoint in same units as avg_temp (integer degrees) */
int gateway_get_setpoint(void);

#endif /* INCLUDE_AWS_MANAGER_H_ */
