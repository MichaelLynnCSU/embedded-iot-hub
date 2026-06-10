/******************************************************************************
 * \file heartbeat.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Device heartbeat monitor interface for BeagleBone data controller.
 *
 * \details Tracks per-device last-seen timestamps and exposes online status
 *          to the shared memory projection layer. heartbeat_stamp() is
 *          called by uart_controller.c on each valid UART frame.
 *          heartbeat_monitor_thread() checks timeouts every second and
 *          updates shared memory and the event log on state change.
 *
 *          hb_timeout_sec[] is static const — each including TU gets its
 *          own copy. Indexed by DEV_ID_E.
 ******************************************************************************/
#ifndef INCLUDE_HEARTBEAT_H_
#define INCLUDE_HEARTBEAT_H_

#include <stdint.h>
#include "sensor_types.h"

void  heartbeat_stamp           (DEV_ID_E idx);
void  heartbeat_snapshot_online (uint8_t *p_out, int count);
void *heartbeat_monitor_thread  (void *p_arg);

static const int hb_timeout_sec[DEV_COUNT] = { 30, 300, 300, 300 };
#endif
