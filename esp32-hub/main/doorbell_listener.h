/******************************************************************************
 * \file doorbell_listener.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief UDP doorbell event listener public interface.
 *
 * \details Receives doorbell press events from ESP32 doorbell cam over UDP.
 *          Parses JSON envelope, validates device_id, publishes to vroom bus.
 *          Runs as a FreeRTOS task — start with doorbell_listener_start().
 *
 *          Lane A only — event path is independent of JPEG delivery.
 *          Neither lane depends on the other for correctness.
 ******************************************************************************/
#ifndef DOORBELL_LISTENER_H
#define DOORBELL_LISTENER_H

/**
 * \brief Start the doorbell UDP listener task.
 *
 * Binds UDP socket on HUB_DOORBELL_UDP_PORT, receives JSON envelopes
 * from ESP32 doorbell cam, publishes DOORBELL_PAYLOAD_T to vroom bus.
 *
 * \warning Call after WiFi manager init, before tasks need doorbell events.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
void doorbell_listener_start(EventGroupHandle_t wifi_eg);

#endif /* DOORBELL_LISTENER_H */
