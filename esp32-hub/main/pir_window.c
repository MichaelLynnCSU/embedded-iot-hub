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
 *          the hold timer is (re)started. g_pir_occupied is 1 for the
 *          duration of PIR_HOLD_SEC and drops to 0 once the hold elapses
 *          with no new threshold crossing.
 *
 *          Window size:  PIR_WINDOW_SEC        (60 s)
 *          Threshold:    PIR_WINDOW_THRESHOLD   (2 events)
 *          Hold:         PIR_HOLD_SEC           (600 s / 10 min)
 *
 *          The 10 s device heartbeat (occ=0) is the resolution floor --
 *          the window cannot meaningfully be smaller than ~20-30 s.
 *          motion_count delta in ble_scan.c can detect missed occ=1
 *          bursts during lossy BLE periods.
 ******************************************************************************/

#include "pir_window.h"
#include "config.h"

#define PIR_EVENT_BUF_SIZE   16u   /**< circular buffer depth for occ=1 timestamps */

static uint32_t g_pir_event_buf[PIR_EVENT_BUF_SIZE]; /**< occ=1 event timestamps ms */
static int      g_pir_event_head   = 0;              /**< next write index */
static uint32_t g_pir_last_hold_ms = 0u;             /**< timestamp when hold last started */
static int      g_pir_occupied     = 0;              /**< 0=empty, 1=occupied */

/*----------------------------------------------------------------------------*/

void pir_window_update(uint32_t now_ms, int occ)
{
   uint32_t window_ms = 0u; /**< window width in ms */
   uint32_t hold_ms   = 0u; /**< hold duration in ms */
   int      count     = 0;  /**< events inside window */
   int      i         = 0;  /**< loop index */

   window_ms = PIR_WINDOW_SEC * 1000u;
   hold_ms   = PIR_HOLD_SEC   * 1000u;

   /* Stamp event into circular buffer on occ=1 */
   if (1 == occ)
   {
      g_pir_event_buf[g_pir_event_head] = now_ms;
      g_pir_event_head = (g_pir_event_head + 1) % (int)PIR_EVENT_BUF_SIZE;
   }

   /* Count events inside the sliding window */
   for (i = 0; i < (int)PIR_EVENT_BUF_SIZE; i++)
   {
      if ((g_pir_event_buf[i] > 0u) &&
          ((now_ms - g_pir_event_buf[i]) <= window_ms))
      {
         count++;
      }
   }

   /* Restart hold timer if threshold met */
   if (count >= (int)PIR_WINDOW_THRESHOLD)
   {
      g_pir_last_hold_ms = now_ms;
   }

   /* Occupied for duration of hold after last threshold crossing */
   if ((g_pir_last_hold_ms > 0u) &&
       ((now_ms - g_pir_last_hold_ms) <= hold_ms))
   {
      g_pir_occupied = 1;
   }
   else
   {
      g_pir_occupied = 0;
   }
}

/*----------------------------------------------------------------------------*/

int pir_window_get_occupied(void)
{
   return g_pir_occupied;
}
