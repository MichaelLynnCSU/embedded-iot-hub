/******************************************************************************
 * \file cam_trigger_ipc.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-29
 *
 * \brief IPC contract between controller (sender) and camera_manager
 *        (receiver) for PIR-triggered camera capture requests.
 *
 * \details controller/pipeline/sensor_dispatch.c detects a PIR 0->1
 *          occupancy transition and sends a CamTriggerRequest over a
 *          UNIX domain datagram socket to camera_manager.
 *
 *          camera_manager receives the request, looks up the camera IP
 *          from its heartbeat registry, verifies the camera is online,
 *          and sends a UDP CAPTURE packet directly to the camera on
 *          port 9091.
 *
 *          Transport: UNIX domain datagram (SOCK_DGRAM, AF_UNIX).
 *          No connection, no blocking, fire-and-forget from sender.
 *          camera_manager owns the socket file lifetime.
 *
 * \note    zone maps to CAM_SLOT on the ESP32-CAM side. Zone 0 maps to
 *          the camera registered as CAM_SLOT=0, and so on up to MAX_CAMS-1.
 *          Zones >= MAX_CAMS are ignored by camera_manager.
 ******************************************************************************/

#ifndef CAM_TRIGGER_IPC_H
#define CAM_TRIGGER_IPC_H

#include <stdint.h>

#define CAM_TRIGGER_SOCK "/tmp/cam_trigger.sock"
#define CAM_UDP_PORT     9091   /**< UDP port on ESP32-CAM for CAPTURE triggers */

/**
 * \brief PIR-triggered capture request — sent by controller,
 *        received by camera_manager.
 */
struct CamTriggerRequest
{
    uint64_t event_id;  /**< pir_slots[i].event_id — correlation key */
    uint8_t  zone;      /**< PIR slot index (0-based) = CAM_SLOT     */
};

#endif /* CAM_TRIGGER_IPC_H */
