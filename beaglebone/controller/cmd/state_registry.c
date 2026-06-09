/******************************************************************************
 * \file state_registry.c
 * \brief Canonical sensor state — single writer authority.
 *
 * \details Owns central_ledger and state_mutex. update_snapshot() is the
 *          only write path. get_snapshot() is the only read path.
 *          No other translation unit touches central_ledger or state_mutex.
 ******************************************************************************/

#include "state_registry.h"

static struct LatestData central_ledger =
{
   .age_pir  = 0xFFFF,
   .age_lgt  = 0xFFFF,
   .age_lck  = 0xFFFF,
   .batt_pir = -1,
   .batt_lck = -1,
};

static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************************
 * \brief Update canonical state from one complete sensor frame.
 *
 * \param p_data - Incoming wire frame. Not modified.
 *
 * \return void
 ******************************************************************************/
void update_snapshot(const struct SensorData *p_data)
{
   int i = 0;

   pthread_mutex_lock(&state_mutex);

   central_ledger.avg_temp           = p_data->avg_temp;
   central_ledger.motion_count       = p_data->motion_count;
   central_ledger.light_state        = p_data->light_state;
   central_ledger.lock_state         = p_data->lock_state;
   central_ledger.timestamp          = p_data->timestamp;
   central_ledger.valid              = 1;
   central_ledger.age_pir            = p_data->age_pir;
   central_ledger.age_lgt            = p_data->age_lgt;
   central_ledger.age_lck            = p_data->age_lck;
   central_ledger.batt_pir           = p_data->batt_pir;
   central_ledger.pir_occupied       = p_data->pir_occupied;
   central_ledger.batt_lck           = p_data->batt_lck;
   central_ledger.batt_motor         = p_data->batt_motor;
   central_ledger.motor_online       = p_data->motor_online;
   central_ledger.doorbell_pressed   = p_data->doorbell_pressed;
   central_ledger.doorbell_device_id = p_data->doorbell_device_id;

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (p_data->reed_slots[i].active)
      {
         central_ledger.reed_slots[i] = p_data->reed_slots[i];
      }
   }

   for (i = 0; i < MAX_PIRS; i++)
   {
      central_ledger.pir_slots[i] = p_data->pir_slots[i];
   }
   central_ledger.pir_count = p_data->pir_count;

   for (i = 0; i < MAX_TEMPS; i++)
   {
      central_ledger.temp_slots[i] = p_data->temp_slots[i];
   }
   central_ledger.temp_count = p_data->temp_count;

   pthread_mutex_unlock(&state_mutex);
}

/******************************************************************************
 * \brief Copy canonical state under lock into caller-supplied struct.
 *
 * \param p_out - Destination. Overwritten entirely. Must not be NULL.
 *
 * \return void
 ******************************************************************************/
void get_snapshot(struct LatestData *p_out)
{
   pthread_mutex_lock(&state_mutex);
   *p_out = central_ledger;
   pthread_mutex_unlock(&state_mutex);
}
