/******************************************************************************
 * \file db_persist.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-09
 *
 * \brief Unit of Work persistence facade for BeagleBone data controller.
 *
 * \details Takes the raw SensorData wire frame and batches db_begin,
 *          db_save_reading, db_save_motor, db_save_reed, and db_commit
 *          into one fsync. No shared memory access, no snapshot
 *          dependency — persistence of ingress data only.
 ******************************************************************************/
#ifndef INCLUDE_CMD_DB_PERSIST_H_
#define INCLUDE_CMD_DB_PERSIST_H_

#include "sensor_types.h"

void db_persist_frame(const struct SensorData *p_data);

#endif
