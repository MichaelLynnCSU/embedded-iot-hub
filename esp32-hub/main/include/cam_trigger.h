#ifndef CAM_TRIGGER_H
#define CAM_TRIGGER_H
/******************************************************************************
 * \file cam_trigger.h
 * \brief ESP32-CAM UDP trigger interface.
 *
 * \details Sends a one-shot UDP "CAPTURE" packet to the ESP32-CAM on a
 *          PIR occupied 0->1 transition. The CAM captures a JPEG and
 *          pushes it to the BeagleBone inference_daemon over persistent TCP.
 *
 *          Identity model (2026-06-18):
 *          The camera trigger is a side effect of a PIR event, not a new
 *          event. The PIR event_id is carried into the trigger and forwarded
 *          in the UDP payload so the full chain is traceable with a single
 *          grep:
 *
 *            [BLE_PIR]    tx_id=144 event_id=101 slot=0
 *            [VROOM]      event_id=101 ingest type=BLE_PIR
 *            [UDP_CAM_TX] cam_tx_id=42 event_id=101 zone=0
 *            [UDP_CAM_RX] cam_tx_id=42 event_id=101
 *            [INFER]      event_id=101 person=1 conf=92
 *
 *          cam_tx_id is a local UDP transport counter owned by
 *          cam_trigger.c — useful for debugging dropped or duplicated
 *          UDP packets independently of the business event identity.
 ******************************************************************************/

#include <stdint.h>

/**
 * \brief Fire a UDP CAPTURE packet to the ESP32-CAM.
 *
 * \param event_id  PIR VROOM event_id that caused this trigger. Carried
 *                  into the UDP payload and logged at [UDP_CAM_TX] so the
 *                  camera lifecycle is traceable back to the originating
 *                  PIR event with a single grep on event_id.
 * \param zone      PIR slot index (0-based) that triggered the capture.
 *                  Logged at [UDP_CAM_TX] for multi-zone disambiguation.
 *
 * Opens a SOCK_DGRAM socket, sends "CAPTURE" to CAM_HOST:CAM_UDP_PORT,
 * and closes immediately. Logs an error on socket failure but does not
 * block or retry. Increments cam_tx_id on every call regardless of
 * socket outcome so the counter reflects trigger attempts, not successes.
 */
void send_cam_trigger(uint64_t event_id, uint8_t zone);

#endif /* CAM_TRIGGER_H */
