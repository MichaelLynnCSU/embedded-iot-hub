/******************************************************************************
 * \file doorbell_pending.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-15
 *
 * \brief Doorbell inference "pending result" correlation layer.
 *
 * \details Extracted from uart_controller.c (2026-06-15 refactor: split
 *          into transport / protocol / correlation / lock-adapter modules,
 *          no behavior change). See doorbell_pending.h for full design
 *          rationale.
 *
 *          This is the only module allowed to "remember" doorbell press ->
 *          inference result correlation state. uart_protocol.c calls
 *          doorbell_pending_check() and doorbell_inject_pending() once per
 *          push cycle; doorbell_pending_mark() is called when a press is
 *          seen but no result is available yet in the same cycle.
 *
 *          g_db_pending_mutex guards all fields below since uart_push_thread
 *          and the TCP sync path (uart_update_frame) run on different
 *          threads and both call these functions.
 *
 *          Sending the result as a standalone uart_push_msg() (previous
 *          approach, removed 2026-06-15) collided with the normal push
 *          bundle on UART and was lost by the STM32 parser — UART is a
 *          single byte stream, two concurrent writes interleave. The result
 *          is always injected into the normal complete bundle instead.
 ******************************************************************************/

#include <string.h>
#include <time.h>
#include <pthread.h>
#include "log.h"
#include "doorbell_pending.h"
#include "doorbell_result_reader.h"
#include "doorbell_result_shm.h"

#define DOORBELL_PENDING_TIMEOUT_SEC 10

static pthread_mutex_t g_db_pending_mutex = PTHREAD_MUTEX_INITIALIZER;
static int             g_db_pending              = 0; /* 1 = press seen, result not yet consumed */
static int             g_db_pending_device       = 0;
static time_t          g_db_pending_since        = 0;
/* Result fields — populated by doorbell_pending_check(), injected by next push cycle */
static int             g_db_pending_result_ready = 0;
static uint64_t        g_db_pending_event_id     = 0;
static uint8_t         g_db_pending_person       = 0;
static uint8_t         g_db_pending_conf_pct     = 0;
static char            g_db_pending_asset[DOORBELL_ASSET_LEN];

/******************************************************************************
 * \brief Mark a doorbell press as pending an inference result.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void doorbell_pending_mark(int device_id)
{
   pthread_mutex_lock(&g_db_pending_mutex);
   g_db_pending        = 1;
   g_db_pending_device = device_id;
   g_db_pending_since  = time(NULL);
   pthread_mutex_unlock(&g_db_pending_mutex);
}

/******************************************************************************
 * \brief Poll for late-arriving doorbell inference result and store it.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void doorbell_pending_check(void)
{
   int      pending    = 0;
   int      device     = 0;
   time_t   since      = 0;
   time_t   now        = time(NULL);
   uint64_t event_id   = 0;
   uint8_t  person     = 0;
   uint8_t  conf_pct   = 0;
   char     asset[DOORBELL_ASSET_LEN] = {0};
   int      got_result = 0;

   pthread_mutex_lock(&g_db_pending_mutex);
   pending = g_db_pending;
   device  = g_db_pending_device;
   since   = g_db_pending_since;
   if (g_db_pending_result_ready)
   {
      /* Result already stored — wait for push cycle to consume it */
      pthread_mutex_unlock(&g_db_pending_mutex);
      return;
   }
   pthread_mutex_unlock(&g_db_pending_mutex);

   if (!pending) { return; }

   got_result = doorbell_result_reader_poll(&event_id, &person, &conf_pct,
                                             asset, sizeof(asset));
   if (got_result)
   {
      LOG("[CONTROLLER] result_ready event_id=" EVENT_ID_FMT
          " device_id=%d person=%d conf=%d",
          EVENT_ID_ARG(event_id), device, person, conf_pct);

      pthread_mutex_lock(&g_db_pending_mutex);
      g_db_pending_result_ready = 1;
      g_db_pending_event_id     = event_id;
      g_db_pending_person       = person;
      g_db_pending_conf_pct     = conf_pct;
      (void)strncpy(g_db_pending_asset, asset, DOORBELL_ASSET_LEN - 1);
      g_db_pending_asset[DOORBELL_ASSET_LEN - 1] = '\0';
      pthread_mutex_unlock(&g_db_pending_mutex);
      return;
   }

   if ((now - since) > DOORBELL_PENDING_TIMEOUT_SEC)
   {
      LOG("[CONTROLLER] result_timeout device_id=%d", device);
      pthread_mutex_lock(&g_db_pending_mutex);
      g_db_pending              = 0;
      g_db_pending_result_ready = 0;
      pthread_mutex_unlock(&g_db_pending_mutex);
   }
}

/******************************************************************************
 * \brief Inject a stored pending inference result into the current push cycle.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void doorbell_inject_pending(int      *p_pressed,
                              int      *p_device_id,
                              int      *p_valid,
                              uint64_t *p_event_id,
                              uint8_t  *p_person,
                              uint8_t  *p_conf_pct,
                              char     *p_asset,
                              int       asset_len)
{
   pthread_mutex_lock(&g_db_pending_mutex);

   if (!(*p_pressed) && g_db_pending_result_ready)
   {
      *p_pressed   = 1;
      *p_device_id = g_db_pending_device;
      *p_valid     = 1;
      *p_event_id  = g_db_pending_event_id;
      *p_person    = g_db_pending_person;
      *p_conf_pct  = g_db_pending_conf_pct;
      (void)strncpy(p_asset, g_db_pending_asset, asset_len - 1);
      p_asset[asset_len - 1]    = '\0';
      g_db_pending              = 0;
      g_db_pending_result_ready = 0;

      LOG("[CONTROLLER] inject_pending device_id=%d person=%d conf=%d",
          *p_device_id, *p_person, *p_conf_pct);
   }

   pthread_mutex_unlock(&g_db_pending_mutex);
}
