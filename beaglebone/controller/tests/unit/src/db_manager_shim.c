#include <stdio.h>
#include <pthread.h>
#include <sqlite3.h>
#include <semaphore.h>

#include "controller_internal.h"

/* Override DB_PATH *after* the header — our definition wins over the
   BeagleBone path defined in controller_internal.h.  The #undef clears
   the header's value; the new #define is what db_manager.c will see only
   when this TU is compiled (the real fix is the CMake compile-definition
   below, but keeping this guard here documents intent and suppresses the
   redefinition warning). */
#undef  DB_PATH
#define DB_PATH ":memory:"

FILE            *log_fp    = NULL;
pthread_mutex_t  log_mutex = PTHREAD_MUTEX_INITIALIZER;
sqlite3         *db        = NULL;
volatile int     running   = 1;

struct LatestData        latest_data = {0};
pthread_mutex_t          data_mutex  = PTHREAD_MUTEX_INITIALIZER;
struct SharedSensorData *shm_data    = NULL;
sem_t                   *shm_sem     = NULL;

const char *dev_names[DEV_COUNT] = { "PIR", "LGT", "LCK", "MOTOR" };

void heartbeat_stamp(DEV_ID_E idx)                { (void)idx; }
void heartbeat_snapshot_online(uint8_t *p, int n) { (void)p; (void)n; }
void uart_sync_lock_state(int s)                  { (void)s; }
int  init_shared_memory(void)                     { return 0; }
