/******************************************************************************
 * \file globals.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-09
 *
 * \brief Process-wide singleton externs for BeagleBone data controller.
 *
 * \details Declares the global objects defined in data_controller.c that
 *          must be visible to all translation units: shared memory pointer,
 *          SQLite handle, run flag, device name table, and the shared
 *          memory initialisation function.
 *
 *          No translation unit other than data_controller.c defines these.
 *          All others include this header and reference them as extern.
 ******************************************************************************/
#ifndef INCLUDE_GLOBALS_H_
#define INCLUDE_GLOBALS_H_
#include <sqlite3.h>
#include "sensor_types.h"
#include "shared_data.h"
extern struct SharedSensorData *shm_data;
extern sqlite3                 *db;
extern volatile int             running;
extern const char              *dev_names[DEV_COUNT];
int init_shared_memory(void);
#endif /* INCLUDE_GLOBALS_H_ */
