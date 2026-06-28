/******************************************************************************
 * \file sensor_dispatch.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-09
 *
 * \brief Sensor frame fanout interface for BeagleBone data controller.
 *
 * \details sensor_frame_dispatch() is the single entry point for every
 *          validated inbound SensorData frame from pipe_reader.c. Routes
 *          the frame to state_registry, shm_updater, db_persist, and
 *          uart_controller subsystems. No pipe I/O, no threading.
 ******************************************************************************/
#ifndef INCLUDE_CMD_SENSOR_DISPATCH_H_
#define INCLUDE_CMD_SENSOR_DISPATCH_H_

#include "sensor_types.h"

void sensor_frame_dispatch(const struct SensorData *p_data);

#endif /* INCLUDE_CMD_SENSOR_DISPATCH_H_ */
