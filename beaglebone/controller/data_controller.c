/******************************************************************************
 * \file data_controller.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief BeagleBone data controller — main entry point.
 *
 * \details Owns all global resources and spawns five worker threads:
 *
 *          - receive_data_thread:    reads SensorData from sensor pipe
 *          - command_handler_thread: reads CommandMsg from command pipe
 *          - uart_reader_thread:     reads STM32 UART data
 *          - heartbeat_monitor_thread: monitors device timeouts
 *          - uart_push_thread:       pushes UART data to sensor pipe
 *
 *          Shared resources:
 *          - shm_data:          POSIX shared memory for LCD display IPC
 *          - shm_data->shm_mutex: process-shared mutex protecting shm_data
 *          - g_uart_frame_sem:  counting semaphore — value equals number of
 *                               frames queued in g_uart_ring
 *          - g_uart_ring:       ring buffer between uart_reader_thread and
 *                               uart_push_thread
 *          - db:                SQLite database handle
 *          - log_fp:            log file handle
 *          - log_mutex:         serialises log writes across all threads
 *          - running:           volatile flag cleared by signal handler
 *
 * \note    Semaphore → mutex + ring buffer (2026-05-22):
 *          shm_sem (named POSIX semaphore, O_CREAT, val=1) removed.
 *          Replaced by shm_data->shm_mutex (PTHREAD_PROCESS_SHARED)
 *          embedded in the shared memory struct — no named object,
 *          no sem_open/sem_close/sem_unlink, LCD process gets the mutex
 *          automatically when it maps the same region.
 *          g_uart_frame_sem added as a process-local counting semaphore
 *          (pshared=0, val=0). g_uart_ring added as the backing store.
 *          Both are initialized here and destroyed in cleanup().
 ******************************************************************************/

#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include "config.h"
#include "sensor_types.h"
#include "shared_data.h"
#include "log.h"
#include "globals.h"
#include "heartbeat.h"
#include "db_manager.h"
#include "uart_controller.h"
#include "cmd/cmd_handler.h"
#include "cmd/pipe_reader.h"

FILE                    *log_fp    = NULL; /**< log file handle */
pthread_mutex_t          log_mutex = PTHREAD_MUTEX_INITIALIZER; /**< log serialiser */
struct SharedSensorData *shm_data  = NULL; /**< shared memory data pointer */
sqlite3                 *db        = NULL; /**< SQLite database handle */
volatile int             running   = 1;    /**< main loop run flag */

/* Counting semaphore — value == frames queued in g_uart_ring.
 * Initialized to 0: no frames pending at startup.
 * Producer: uart_parse_line() calls sem_post() after uart_ring_push().
 * Consumer: uart_push_thread() calls sem_timedwait() before uart_ring_pop(). */
sem_t       g_uart_frame_sem;

/* Ring buffer — single producer (uart_reader_thread),
 * single consumer (uart_push_thread). */
uart_ring_t g_uart_ring;

/******************************************************************************
 * \brief POSIX signal handler — initiates graceful shutdown.
 *
 * \param sig - Signal number received.
 *
 * \return void
 *
 * \details Clears running flag to stop all thread loops.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void signal_handler(int sig)
{
   LOG_WRN("Signal %d received, shutting down", sig);
   running = 0;

   /* Wake uart_push_thread immediately so it can observe running == 0
    * rather than blocking until the sem_timedwait deadline expires. */
   sem_post(&g_uart_frame_sem);
}

/******************************************************************************
 * \brief Initialize POSIX shared memory and process-shared mutex.
 *
 * \return int - 0 on success, -1 on failure.
 *
 * \details Unlinks any existing shared memory, creates and maps a new
 *          region, zeroes it, then initializes shm_data->shm_mutex with
 *          PTHREAD_PROCESS_SHARED so the LCD process can lock it after
 *          mapping the same region.
 *
 *          Also initializes g_uart_frame_sem (pshared=0, val=0) and
 *          g_uart_ring for the UART frame producer/consumer path.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int init_shared_memory(void)
{
   int                 shm_fd = -1; /**< shared memory file descriptor */
   pthread_mutexattr_t attr;        /**< mutex attribute for PROCESS_SHARED */

   (void)shm_unlink(SHM_NAME);

   shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
   if (0 > shm_fd)
   {
      LOG_ERR("shm_open failed");
      return -1;
   }

   (void)ftruncate(shm_fd, sizeof(struct SharedSensorData));

   shm_data = mmap(NULL,
                   sizeof(struct SharedSensorData),
                   PROT_READ | PROT_WRITE,
                   MAP_SHARED,
                   shm_fd, 0);

   if (MAP_FAILED == shm_data)
   {
      LOG_ERR("mmap failed");
      return -1;
   }

   (void)memset(shm_data, 0, sizeof(struct SharedSensorData));

   /* Initialize the process-shared mutex embedded in the shared region.
    * PTHREAD_PROCESS_SHARED: any process that maps this region can lock it.
    * Must be called before the LCD process maps the region. */
   pthread_mutexattr_init(&attr);
   pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);

   if (0 != pthread_mutex_init(&shm_data->shm_mutex, &attr))
   {
      LOG_ERR("shm_mutex init failed");
      pthread_mutexattr_destroy(&attr);
      return -1;
   }

   pthread_mutexattr_destroy(&attr);

   /* Counting semaphore — process-local (pshared=0), starts at 0.
    * Value tracks frames queued in g_uart_ring exactly. */
   if (0 != sem_init(&g_uart_frame_sem, 0, 0))
   {
      LOG_ERR("g_uart_frame_sem init failed");
      return -1;
   }

   /* Ring buffer — zero head/tail, mutex default-initialized. */
   (void)memset(&g_uart_ring, 0, sizeof(g_uart_ring));
   pthread_mutex_init(&g_uart_ring.mutex, NULL);

   LOG_INF("Shared memory and semaphores initialized");
   return 0;
}

/******************************************************************************
 * \brief Spawn all controller worker threads.
 *
 * \param p_rx_thread   - Output thread handle for receive_data_thread.
 * \param p_cmd_thread  - Output thread handle for command_handler_thread.
 * \param p_uart_thread - Output thread handle for uart_reader_thread.
 * \param p_hb_thread   - Output thread handle for heartbeat_monitor_thread.
 * \param p_push_thread - Output thread handle for uart_push_thread.
 *
 * \return int - 0 on success, -1 if any thread creation fails.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static int create_threads(pthread_t *p_rx_thread,
                           pthread_t *p_cmd_thread,
                           pthread_t *p_uart_thread,
                           pthread_t *p_hb_thread,
                           pthread_t *p_push_thread)
{
   int ret = 0; /**< pthread return value */

   ret = pthread_create(p_rx_thread, NULL, receive_data_thread, NULL);
   if (0 != ret)
   {
      LOG_ERR("receive_data_thread create failed (err=%d)", ret);
      return -1;
   }

   ret = pthread_create(p_cmd_thread, NULL, command_handler_thread, NULL);
   if (0 != ret)
   {
      LOG_ERR("command_handler_thread create failed (err=%d)", ret);
      return -1;
   }

   ret = pthread_create(p_uart_thread, NULL, uart_reader_thread, NULL);
   if (0 != ret)
   {
      LOG_ERR("uart_reader_thread create failed (err=%d)", ret);
      return -1;
   }

   ret = pthread_create(p_hb_thread, NULL, heartbeat_monitor_thread, NULL);
   if (0 != ret)
   {
      LOG_ERR("heartbeat_monitor_thread create failed (err=%d)", ret);
      return -1;
   }

   ret = pthread_create(p_push_thread, NULL, uart_push_thread, NULL);
   if (0 != ret)
   {
      LOG_ERR("uart_push_thread create failed (err=%d)", ret);
      return -1;
   }

   return 0;
}

/******************************************************************************
 * \brief Clean up all global resources on shutdown.
 *
 * \return void
 *
 * \details Destroys shm_mutex inside the mapped region, then unmaps and
 *          unlinks shared memory. Destroys g_uart_frame_sem and the ring
 *          buffer mutex. Closes SQLite database and log file.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void cleanup(void)
{
   if (NULL != shm_data)
   {
      pthread_mutex_destroy(&shm_data->shm_mutex);
      (void)munmap(shm_data, sizeof(struct SharedSensorData));
      (void)shm_unlink(SHM_NAME);
   }

   sem_destroy(&g_uart_frame_sem);
   pthread_mutex_destroy(&g_uart_ring.mutex);

   if (NULL != db)
   {
      sqlite3_close(db);
   }

   if (NULL != log_fp)
   {
      fclose(log_fp);
   }
}

/******************************************************************************
 * \brief Data controller application entry point.
 *
 * \return int - 0 on clean exit, 1 on initialization failure.
 *
 * \details Opens log file, registers signal handlers, initializes shared
 *          memory and semaphores, database, and sensor pipe, then spawns
 *          worker threads. Joins all threads before cleanup and exit.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int main(void)
{
   pthread_t rx_thread;   /**< receive data thread handle */
   pthread_t cmd_thread;  /**< command handler thread handle */
   pthread_t uart_thread; /**< UART reader thread handle */
   pthread_t hb_thread;   /**< heartbeat monitor thread handle */
   pthread_t push_thread; /**< UART push thread handle */

   log_fp = fopen(CONTROLLER_LOG, "a");
   if (NULL == log_fp)
   {
      perror("log open failed");
      return 1;
   }

   LOG_INF("=========================================");
   LOG_INF("Data Controller starting");
   LOG_INF("=========================================");

   (void)signal(SIGINT,  signal_handler);
   (void)signal(SIGTERM, signal_handler);

   if (0 > init_shared_memory())
   {
      LOG_ERR("Shared memory init failed");
      fclose(log_fp);
      return 1;
   }

   db_open_and_init();

   (void)unlink(SENSOR_PIPE);
   (void)mkfifo(SENSOR_PIPE, 0666);

   if (0 > create_threads(&rx_thread,
                           &cmd_thread,
                           &uart_thread,
                           &hb_thread,
                           &push_thread))
   {
      LOG_ERR("Thread creation failed");
      cleanup();
      return 1;
   }

   LOG_INF("All threads started, controller ready");

   (void)pthread_join(rx_thread,   NULL);
   (void)pthread_join(cmd_thread,  NULL);
   (void)pthread_join(uart_thread, NULL);
   (void)pthread_join(hb_thread,   NULL);
   (void)pthread_join(push_thread, NULL);

   LOG_INF("Controller shutting down");

   cleanup();

   return 0;
}
