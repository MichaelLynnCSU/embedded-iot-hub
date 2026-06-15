/******************************************************************************
 * \file doorbell_result_reader.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief Accessor module for the /doorbell_result shared memory segment.
 *
 * \details See doorbell_result_reader.h. Attaches read-write to the
 *          segment created by doorbell_daemon (this module never creates
 *          it — O_CREAT is intentionally absent). Implements the
 *          lock-free publish/consume contract from doorbell_result_shm.h:
 *          compares event_id against a local last-seen value, copies out
 *          the other fields on change with a memory barrier in between.
 *
 * \note    event_id tracing (2026-06-15):
 *          doorbell_result_reader_poll() now returns event_id to the
 *          caller via p_event_id (shm struct itself is unchanged — the
 *          field already existed, just wasn't surfaced). On a successful
 *          consume (return 1), this module logs
 *          "[SHM] -> [CONTROLLER] consume event_id=... ..." exactly once,
 *          using the values that were just copied out — not a re-read of
 *          shared memory — so the log always reflects what the caller
 *          actually received. This is the single log point for "consume";
 *          callers should not duplicate it. Callers are responsible for
 *          their own "waiting_for_result" / "send" logging, since only
 *          the caller knows why it polled and what it does with the
 *          result.
 ******************************************************************************/
#include "doorbell_result_reader.h"
#include "doorbell_result_shm.h"
#include "log.h"
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/*---------------------------------------------------------------------------*/
/* Module state                                                                */
/*---------------------------------------------------------------------------*/
static struct DoorbellResult *g_result         = NULL; /**< mapped segment, or NULL if not yet attached */
static uint64_t                g_last_event_id = 0;     /**< last event_id consumed (local to this reader) */

/*---------------------------------------------------------------------------*/
/* Public API                                                                  */
/*---------------------------------------------------------------------------*/
int doorbell_result_reader_init(void)
{
   int fd = -1;

   if (NULL != g_result) { return 0; } /* already attached */

   /* No O_CREAT — doorbell_daemon owns creation. If it hasn't started
    * yet, shm_open fails and we simply remain not-attached; poll()
    * will retry via this same function on a later call. */
   fd = shm_open(DOORBELL_RESULT_SHM_NAME, O_RDWR, 0666);
   if (0 > fd)
   {
      return -1;
   }

   g_result = mmap(NULL, sizeof(struct DoorbellResult),
                    PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
   close(fd);

   if (MAP_FAILED == g_result)
   {
      g_result = NULL;
      return -1;
   }

   return 0;
}

/**
 * \brief Poll /doorbell_result for a new (not-yet-consumed) inference
 *        result.
 *
 * \details Compares the shm segment's event_id against this reader's
 *          locally-held last-seen value. If unchanged, returns 0 with no
 *          side effects (no log — callers decide whether/how to log a
 *          miss, since only the caller knows whether it's actively
 *          waiting on a result or just polling opportunistically).
 *
 *          If changed: issues a memory barrier, copies out event_id,
 *          person, conf_pct, and asset into local variables, then copies
 *          those local variables (not a re-read of shm) out to the
 *          caller's pointers and into the "[SHM] -> [CONTROLLER] consume"
 *          log line. Updates g_last_event_id last, so the log always
 *          matches exactly what the caller receives.
 *
 * \param[out] p_event_id  Event ID of the consumed result (may be NULL).
 * \param[out] p_person    1 if a person was detected, else 0 (may be NULL).
 * \param[out] p_conf_pct  Confidence 0-100 (may be NULL).
 * \param[out] p_asset     Buffer for the asset timestamp string (may be
 *                          NULL if asset_len == 0).
 * \param[in]  asset_len   Size of p_asset buffer, including space for the
 *                          terminating nul.
 *
 * \return 1 if a new result was consumed and copied out, 0 if no new
 *         result is available (or the segment isn't attached yet).
 */
int doorbell_result_reader_poll(uint64_t *p_event_id,
                                 uint8_t *p_person, uint8_t *p_conf_pct,
                                 char *p_asset, size_t asset_len)
{
   uint64_t event_id;
   uint8_t  device_id;
   uint8_t  person;
   uint8_t  conf_pct;
   char     asset[20];

   if (NULL == g_result)
   {
      /* Retry attach lazily — doorbell_daemon may have started after us. */
      if (0 != doorbell_result_reader_init())
      {
         return 0;
      }
   }

   if (g_result->event_id == g_last_event_id)
   {
      return 0;
   }

   __sync_synchronize();

   /* Copy everything out of shm once, into locals. All subsequent use
    * (caller out-params and the consume log) reads from these locals,
    * not from g_result again, so the log reflects exactly what the
    * caller receives even if the writer publishes again concurrently. */
   event_id  = g_result->event_id;
   device_id = g_result->device_id;
   person    = g_result->person;
   conf_pct  = g_result->conf_pct;
   strncpy(asset, g_result->asset, sizeof(asset) - 1);
   asset[sizeof(asset) - 1] = '\0';

   if (NULL != p_event_id) { *p_event_id = event_id; }
   if (NULL != p_person)   { *p_person   = person;   }
   if (NULL != p_conf_pct) { *p_conf_pct = conf_pct; }
   if (NULL != p_asset && asset_len > 0)
   {
      strncpy(p_asset, asset, asset_len - 1);
      p_asset[asset_len - 1] = '\0';
   }

   LOG("[SHM] -> [CONTROLLER] consume event_id=" EVENT_ID_FMT
       " device_id=%d person=%d conf=%d asset=%s",
       EVENT_ID_ARG(event_id), device_id, person, conf_pct, asset);

   g_last_event_id = event_id;
   return 1;
}
