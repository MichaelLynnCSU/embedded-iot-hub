/******************************************************************************
 * \file    pir_window.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   PIR occupancy sliding-window implementation.
 *
 * \details Implements a time-based sliding window over occ=1 events
 *          received from the PIR_Motion BLE advertisement stream.
 *
 *          On each call with occ=1 the current timestamp is written into
 *          a circular buffer (PIR_EVENT_BUF_SIZE deep). The window then
 *          counts how many buffered timestamps fall within the last
 *          PIR_WINDOW_SEC seconds. If the count meets PIR_WINDOW_THRESHOLD
 *          the hold timer is (re)started. occupied is 1 for the duration
 *          of PIR_HOLD_SEC and drops to 0 once the hold elapses with no
 *          new threshold crossing.
 *
 *          Window size:  PIR_WINDOW_SEC        (60 s)
 *          Threshold:    PIR_WINDOW_THRESHOLD   (2 events)
 *          Hold:         PIR_HOLD_SEC           (600 s / 10 min)
 *
 * \note    Per-slot occupancy (2026-05-20):
 *          Single-window globals (g_pir_event_buf, g_pir_event_head,
 *          g_pir_last_hold_ms, g_pir_occupied) replaced by arrays
 *          indexed by slot (0..MAX_PIRS-1). pir_window_update() and
 *          pir_window_get_occupied() both take a slot argument.
 *          Call site in ble_scan.c handle_pir_dynamic() passes the
 *          allocated slot index. ble_manager.c ble_get_pir_occupied()
 *          updated to take slot argument and forward here.
 ******************************************************************************/

#include "pir_window.h"
#include "config.h"

#define PIR_EVENT_BUF_SIZE   16u   /**< circular buffer depth per slot */

static uint32_t g_pir_event_buf[MAX_PIRS][PIR_EVENT_BUF_SIZE]; /**< per-slot event timestamps ms */
static int      g_pir_event_head[MAX_PIRS];                     /**< per-slot next write index    */
static uint32_t g_pir_last_hold_ms[MAX_PIRS];                   /**< per-slot hold start time ms  */
static int      g_pir_occupied[MAX_PIRS];                       /**< per-slot occupied flag       */

/*----------------------------------------------------------------------------*/

void pir_window_update(int slot, uint32_t now_ms, int occ)
{
   uint32_t window_ms = 0u; /**< window width in ms */
   uint32_t hold_ms   = 0u; /**< hold duration in ms */
   int      count     = 0;  /**< events inside window */
   int      i         = 0;  /**< loop index */

   if ((slot < 0) || (slot >= (int)MAX_PIRS))
   {
      return;
   }

   window_ms = PIR_WINDOW_SEC * 1000u;
   hold_ms   = PIR_HOLD_SEC   * 1000u;

   /* Stamp event into circular buffer on occ=1 */
   if (1 == occ)
   {
      g_pir_event_buf[slot][g_pir_event_head[slot]] = now_ms;
      g_pir_event_head[slot] =
         (g_pir_event_head[slot] + 1) % (int)PIR_EVENT_BUF_SIZE;
   }

   /* Count events inside the sliding window */
   for (i = 0; i < (int)PIR_EVENT_BUF_SIZE; i++)
   {
      if ((g_pir_event_buf[slot][i] > 0u) &&
          ((now_ms - g_pir_event_buf[slot][i]) <= window_ms))
      {
         count++;
      }
   }

   /* Restart hold timer if threshold met */
   if (count >= (int)PIR_WINDOW_THRESHOLD)
   {
      g_pir_last_hold_ms[slot] = now_ms;
   }

   /* Occupied for duration of hold after last threshold crossing */
   if ((g_pir_last_hold_ms[slot] > 0u) &&
       ((now_ms - g_pir_last_hold_ms[slot]) <= hold_ms))
   {
      g_pir_occupied[slot] = 1;
   }
   else
   {
      g_pir_occupied[slot] = 0;
   }
}

/*----------------------------------------------------------------------------*/

int pir_window_get_occupied(int slot)
{
   if ((slot < 0) || (slot >= (int)MAX_PIRS))
   {
      return 0;
   }

   return g_pir_occupied[slot];
}
