#ifndef INCLUDE_DB_MANAGER_H_
#define INCLUDE_DB_MANAGER_H_

#include "sensor_types.h"
#include "../include/shared_data.h"

void db_open_and_init(void);
void db_begin(void);
void db_commit(void);
void db_rollback(void);
void db_save_uart    (const char *p_dev_id, int val, int batt);
void db_save_event   (const char *p_dev_name, const char *p_event);
void db_save_reading (const struct SensorData *p_data);
void db_save_reed    (int slot, const char *p_name, int state, int batt, int age);
void db_save_motor   (int online, int batt);
int  db_query_rooms  (struct RoomStatus *p_out, int max_rooms);

#endif
