#include "log.h"
#include "db_manager.h"
#include <stdio.h>
/******************************************************************************
 * \file uart_staging.c
 * \brief UART protocol transitions — partial state updaters and reed logic.
 *
 * \details Provides UART-specific partial writes into the canonical state
 *          registry and owns reed slot transition detection. Does not own
 *          any state — all writes go through state_registry.
 ******************************************************************************/

#include "uart_staging.h"
#include "state_registry.h"

#define REED_DEV_BUF_SIZE   16
#define REED_EVENT_BUF_SIZE 64

static void check_reed_generation(int slot, uint16_t old_gen, uint16_t new_gen)
{
   char ev[REED_EVENT_BUF_SIZE] = {0};
   char dev[REED_DEV_BUF_SIZE]  = {0};

   if ((old_gen > 0) && (new_gen != old_gen))
   {
      (void)snprintf(ev,  sizeof(ev),
                     "gen %u->%u (device replaced)",
                     old_gen, new_gen);
      (void)snprintf(dev, sizeof(dev), "REED%d", slot + 1);
      LOG("[EVENT] %s %s", dev, ev);
      db_save_event(dev, ev);
   }
}

static void check_reed_online_state(int     slot,
                                    uint8_t was_offline,
                                    uint8_t now_offline)
{
   char dev[REED_DEV_BUF_SIZE] = {0};

   (void)snprintf(dev, sizeof(dev), "REED%d", slot + 1);

   if (!was_offline && now_offline)
   {
      LOG("[EVENT] %s went offline", dev);
      db_save_event(dev, "offline");
   }
   else if (was_offline && !now_offline)
   {
      LOG("[EVENT] %s back online", dev);
      db_save_event(dev, "online");
   }
}

/******************************************************************************
 * \brief Detect reed slot generation and online/offline transitions.
 *
 * \param p_data - Incoming wire frame to compare against current registry.
 *
 * \details Reads current reed state via get_snapshot() for comparison,
 *          then logs and persists any transitions detected. The registry
 *          update itself is handled by update_snapshot() in state_registry.
 ******************************************************************************/
void uart_check_reeds(const struct SensorData *p_data)
{
   struct LatestData current;
   int               i = 0;

   get_snapshot(&current);

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (!p_data->reed_slots[i].active) { continue; }

      check_reed_generation(i,
                            current.reed_slots[i].gen,
                            p_data->reed_slots[i].gen);

      check_reed_online_state(i,
                              current.reed_slots[i].offline,
                              p_data->reed_slots[i].offline);

      LOG("Reed %d (%s): state=%d batt=%d%% age=%d",
          i + 1,
          p_data->reed_slots[i].name,
          p_data->reed_slots[i].state,
          p_data->reed_slots[i].batt,
          p_data->reed_slots[i].age);
   }
}

void uart_stage_lock(int lock_state, int batt)
{
   struct SensorData partial = {0};
   struct LatestData current;

   /* Read current state, overlay lock fields, write back. */
   get_snapshot(&current);

   partial.avg_temp     = current.avg_temp;
   partial.motion_count = current.motion_count;
   partial.light_state  = current.light_state;
   partial.lock_state   = lock_state;
   partial.timestamp    = current.timestamp;
   partial.batt_lck     = (int8_t)batt;
   partial.batt_motor   = current.batt_motor;
   partial.motor_online = current.motor_online;
   partial.age_pir      = current.age_pir;
   partial.age_lgt      = current.age_lgt;
   partial.age_lck      = current.age_lck;
   partial.batt_pir     = current.batt_pir;

   update_snapshot(&partial);
}

void uart_stage_pir(int val)
{
   struct SensorData partial = {0};
   struct LatestData current;

   get_snapshot(&current);

   partial.avg_temp     = current.avg_temp;
   partial.motion_count = val;
   partial.light_state  = current.light_state;
   partial.lock_state   = current.lock_state;
   partial.timestamp    = current.timestamp;
   partial.batt_lck     = current.batt_lck;
   partial.batt_motor   = current.batt_motor;
   partial.motor_online = current.motor_online;
   partial.age_pir      = current.age_pir;
   partial.age_lgt      = current.age_lgt;
   partial.age_lck      = current.age_lck;
   partial.batt_pir     = current.batt_pir;

   update_snapshot(&partial);
}

void uart_stage_light(int val)
{
   struct SensorData partial = {0};
   struct LatestData current;

   get_snapshot(&current);

   partial.avg_temp     = current.avg_temp;
   partial.motion_count = current.motion_count;
   partial.light_state  = val;
   partial.lock_state   = current.lock_state;
   partial.timestamp    = current.timestamp;
   partial.batt_lck     = current.batt_lck;
   partial.batt_motor   = current.batt_motor;
   partial.motor_online = current.motor_online;
   partial.age_pir      = current.age_pir;
   partial.age_lgt      = current.age_lgt;
   partial.age_lck      = current.age_lck;
   partial.batt_pir     = current.batt_pir;

   update_snapshot(&partial);
}
