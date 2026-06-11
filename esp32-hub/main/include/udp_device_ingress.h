/******************************************************************************
 * \file udp_device_ingress.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief UDP device ingress public interface.
 *
 * \details Single UDP socket receives JSON envelopes from all ESP32
 *          devices. Routes by device_type field. Runs as FreeRTOS task.
 *
 *          Handles:
 *          - doorbell presses and heartbeats (device_type: "doorbell")
 *          - camera heartbeats              (device_type: "cam")
 *
 *          JSON envelope:
 *          {"device_id":N,"device_type":"doorbell"|"cam",
 *           "event_type":"press"|"heartbeat","timestamp_ms":NNNN}
 ******************************************************************************/
#ifndef UDP_DEVICE_INGRESS_H
#define UDP_DEVICE_INGRESS_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <stdbool.h>
#include <stdint.h>

/*************************** FUNCTION PROTOTYPES ******************************/

/** \brief Start the UDP device ingress task.
 *  \param wifi_eg - WiFi event group handle, waits for WIFI_CONNECTED_BIT. */
void udp_device_ingress_start(EventGroupHandle_t wifi_eg);

/** \brief Get seconds since doorbell device last seen.
 *  \param device_id - Doorbell slot index 0..MAX_DOORBELL_CAMS-1.
 *  \return uint16_t - Age in seconds, 0xFFFF if never seen. */
uint16_t doorbell_get_age_s(uint8_t device_id);

/** \brief Check if doorbell device is alive within heartbeat threshold.
 *  \param device_id - Doorbell slot index.
 *  \return bool - true if alive. */
bool doorbell_is_alive(uint8_t device_id);

/** \brief Get seconds since camera last sent heartbeat.
 *  \param slot - Camera slot index 0..MAX_CAMS-1.
 *  \return uint16_t - Age in seconds, 0xFFFF if never seen. */
uint16_t cam_get_age_s(uint8_t slot);

/** \brief Check if camera is alive within heartbeat threshold.
 *  \param slot - Camera slot index.
 *  \return bool - true if alive. */
bool cam_is_alive(uint8_t slot);

#endif /* UDP_DEVICE_INGRESS_H */
