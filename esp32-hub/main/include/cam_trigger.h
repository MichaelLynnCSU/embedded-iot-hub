#ifndef CAM_TRIGGER_H
#define CAM_TRIGGER_H

/******************************************************************************
 * \file cam_trigger.h
 * \brief ESP32-CAM UDP trigger interface.
 *
 * \details Sends a one-shot UDP "CAPTURE" packet to the ESP32-CAM on a
 *          PIR occupied 0->1 transition.  The CAM captures a JPEG and
 *          pushes it to the BeagleBone inference_daemon over persistent TCP.
 ******************************************************************************/

/**
 * \brief Fire a UDP CAPTURE packet to the ESP32-CAM.
 *
 * Opens a SOCK_DGRAM socket, sends "CAPTURE" to CAM_HOST:CAM_UDP_PORT,
 * and closes immediately.  Logs an error on socket failure but does not
 * block or retry.
 */
void send_cam_trigger(void);

#endif /* CAM_TRIGGER_H */
