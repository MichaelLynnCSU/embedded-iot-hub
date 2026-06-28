/******************************************************************************
 * \file shm_updater.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-09
 *
 * \brief Shared memory projection layer for BeagleBone data controller.
 *
 * \details Consumes frozen LatestData snapshots from get_snapshot() and
 *          writes into SharedSensorData under shm_mutex. Never touches
 *          central_ledger or state_mutex directly. Also handles command
 *          responses for CMD_GET_LATEST, CMD_GET_DEVICE_STATUS, and
 *          CMD_GET_ROOM_STATUS.
 *
 *          SHM architecture — two writers, three readers:
 *
 *          Writers:
 *            pipe_ingress:    sensor_frame_dispatch() → shm_update_frame()
 *                             → handle_get_latest(p_snapshot, "pipe_ingress")
 *            uart_ingress:    uart_parse_line() → handle_get_latest()
 *                             → handle_get_latest(p_snapshot, "uart_ingress")
 *
 *          Readers:
 *            thermostat_lcd:  thermostat_lcd.c display update loop.
 *                             Reads current_temp, batt_motor,
 *                             device_online[3], data_valid, sequence.
 *                             Logs: [SHM] transport=sensor_shm read
 *                                         dst=thermostat_lcd
 *
 *            inference.py:    PIR-triggered inference daemon (Yocto
 *                             recipes-apps/inference). Reads
 *                             current_motion at a fixed struct offset
 *                             to detect PIR motion increments. Does NOT
 *                             use /doorbell_result — that segment is
 *                             exclusively the doorbell path.
 *
 *            uart_controller: doorbell_pressed consume-and-clear ONLY
 *                             (uart_push_thread and uart_update_frame).
 *                             All sensor state (temp, motion, lgt, lck,
 *                             valid, slots) now comes from get_snapshot()
 *                             via state_registry.c. cam_online[] is read
 *                             directly from p_raw_frame->cam_slots[].
 *                             SHM mutex held only for the one-shot
 *                             doorbell flag clear — one job, minimal
 *                             hold time. See uart_controller.c header
 *                             note "SHM read path (dst=blackpill_lcd)"
 *                             (2026-06-19 cleanup).
 *
 *          Log lines emitted by this module:
 *            [SHM] transport=sensor_shm write src=pipe_ingress ...
 *            [SHM] transport=sensor_shm write src=uart_ingress ...
 *            [SHM] transport=sensor_shm write src=pipe_ingress device=PIR slot=N eid=...
 *            [SHM] transport=sensor_shm write src=pipe_ingress device=REED slot=N eid=...
 *            [SHM] transport=sensor_shm write src=pipe_ingress device=TEMP slot=N eid=...
 *            [SHM] transport=sensor_shm write src=pipe_ingress device=LOCK eid=...
 *            [SHM] transport=sensor_shm write src=pipe_ingress device=LIGHT eid=...
 *            [SHM] get_device_status
 *            [SHM] get_room_status
 *            [SHM] doorbell_press
 ******************************************************************************/
#ifndef INCLUDE_CMD_SHM_UPDATER_H_
#define INCLUDE_CMD_SHM_UPDATER_H_
#include "sensor_types.h"
#include "globals.h"
#include "../../include/shared_data.h"

void shm_update_frame         (const struct LatestData *p_snapshot,
                                const struct SensorData *p_data);
void handle_get_latest        (const struct LatestData *p_snapshot,
                                const char              *p_src);
#endif
