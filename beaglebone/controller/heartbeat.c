/******************************************************************************
 * \file heartbeat.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Device heartbeat monitor for BeagleBone data controller.
 *
 * \details Tracks per-device last-seen timestamps and monitors online/
 *          offline transitions. Stamps are written by uart_controller.c
 *          on each valid UART frame. Monitor thread checks every second
 *          and updates shared memory and database on state change.
 *
 *          Thread safety:
 *          - dev_mutex protects all reads and writes to devices[]
 *          - shm_sem protects shared memory updates
 ******************************************************************************/

#include "controller_internal.h"

#define HB_POLL_SEC  1  /**< heartbeat monitor poll interval seconds */

/** \brief Device name strings — indexed by DEV_ID_E */
const char *dev_names[DEV_COUNT] =
{
   "PIR", "LIGHT", "LOCK", "MOTOR"
};

/** \brief Per-device heartbeat state entry */
typedef struct
{
   time_t  last_seen; /*!< Unix timestamp of last stamp, 0=never */
   uint8_t online;    /*!< 1=online, 0=offline */
} DEVICE_HB_T;

static DEVICE_HB_T     g_devices[DEV_COUNT];                    /**< heartbeat table */
static pthread_mutex_t g_dev_mutex = PTHREAD_MUTEX_INITIALIZER; /**< devices mutex */

/******************************************************************************
 * \brief Record a heartbeat timestamp for a device.
 *
 * \param idx - Device index from DEV_ID_E.
 *
 * \return void
 *
 * \details Called by uart_controller.c on each valid UART frame.
 *          No-op if idx is out of range.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void heartbeat_stamp(DEV_ID_E idx)
{
   if ((idx < 0) || (idx >= DEV_COUNT))
   {
      return;
   }

   pthread_mutex_lock(&g_dev_mutex);
   g_devices[idx].last_seen = time(NULL);
   pthread_mutex_unlock(&g_dev_mutex);
}

/******************************************************************************
 * \brief Copy device online flags into output array.
 *
 * \param p_out - Output array of online flags, sized at least count.
 * \param count - Number of devices to snapshot.
 *
 * \return void
 *
 * \details Called by handle_get_device_status() in cmd_handler.c to
 *          populate shared memory device_online array.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void heartbeat_snapshot_online(uint8_t *p_out, int count)
{
   int i = 0; /**< loop index */

   pthread_mutex_lock(&g_dev_mutex);

   for (i = 0; (i < count) && (i < DEV_COUNT); i++)
   {
      p_out[i] = g_devices[i].online;
   }

   pthread_mutex_unlock(&g_dev_mutex);
}

/******************************************************************************
 * \brief Thread — monitors device heartbeat timeouts each second.
 *
 * \param p_arg - Unused thread argument.
 *
 * \return void* - Always returns NULL.
 *
 * \details Checks all devices every HB_POLL_SEC. On online/offline
 *          transition: logs event, saves to database, and updates
 *          shared memory device_online array and sequence counter.
 *          Runs until running flag is cleared.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void *heartbeat_monitor_thread(void *p_arg)
{
   time_t   now       = 0;    /**< current timestamp */
   int      i         = 0;    /**< loop index */
   uint8_t  is_online = 0;    /**< computed online status */
   const char *p_ev   = NULL; /**< event string pointer */

   (void)p_arg;

   LOG("[HB] Monitor thread started (per-device timeouts: PIR=%ds LGT=%ds LCK=%ds MTR=%ds)",
       hb_timeout_sec[DEV_PIR], hb_timeout_sec[DEV_LIGHT],
       hb_timeout_sec[DEV_LOCK], hb_timeout_sec[DEV_MOTOR]);

   while (running)
   {
      sleep(HB_POLL_SEC);

      now = time(NULL);

      pthread_mutex_lock(&g_dev_mutex);

      for (i = 0; i < DEV_COUNT; i++)
      {
         is_online = (uint8_t)((0 < g_devices[i].last_seen) &&
                               ((now - g_devices[i].last_seen) < hb_timeout_sec[i]));

         if (is_online != g_devices[i].online)
         {
            g_devices[i].online = is_online;
            p_ev = is_online ? "ONLINE" : "OFFLINE";

            LOG("[HB] %-6s went %s", dev_names[i], p_ev);
            db_save_event(dev_names[i], p_ev);

            sem_wait(shm_sem);
            shm_data->device_online[i] = is_online;
            shm_data->sequence++;
            sem_post(shm_sem);
         }
      }

      pthread_mutex_unlock(&g_dev_mutex);
   }

   LOG("[HB] Monitor thread exiting");
   return NULL;
}
