#include "db_manager.h"
/******************************************************************************
 * \file db_persist.c
 * \brief DB persistence facade — persists raw ingress frame, not read model.
 ******************************************************************************/

#include "db_persist.h"

void db_persist_frame(const struct SensorData *p_data)
{
   int i = 0;

   db_begin();

   db_save_reading(p_data);

   db_save_motor(p_data->motor_online, p_data->batt_motor);

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (p_data->reed_slots[i].active)
      {
         db_save_reed(i + 1,
                      p_data->reed_slots[i].name,
                      p_data->reed_slots[i].state,
                      p_data->reed_slots[i].batt,
                      p_data->reed_slots[i].age);
      }
   }

   db_commit();
}
