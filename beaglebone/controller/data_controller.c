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
 *          - shm_data:  POSIX shared memory for LCD display IPC
 *          - shm_sem:   POSIX semaphore protecting shm_data
 *          - db:        SQLite database handle
 *          - log_fp:    log file handle
 *          - log_mutex: serialises log writes across all threads
 *          - running:   volatile flag cleared by signal handler
 ******************************************************************************/

#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>
#include "controller_internal.h"

FILE                    *log_fp    = NULL; /**< log file handle */
pthread_mutex_t          log_mutex = PTHREAD_MUTEX_INITIALIZER; /**< log serialiser */
struct SharedSensorData *shm_data  = NULL; /**< shared memory data pointer */
sem_t                   *shm_sem   = NULL; /**< shared memory semaphore */
sqlite3                 *db        = NULL; /**< SQLite database handle */
volatile int             running   = 1;    /**< main loop run flag */

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
}

/******************************************************************************
 * \brief Initialize POSIX shared memory and semaphore.
 *
 * \return int - 0 on success, -1 on failure.
 *
 * \details Unlinks any existing shared memory and semaphore, creates new
 *          ones, maps shared memory, zeroes it, and opens the semaphore
 *          with initial value 1.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int init_shared_memory(void)
{
   int shm_fd = -1; /**< shared memory file descriptor */

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

   (void)sem_unlink(SEM_NAME);

   shm_sem = sem_open(SEM_NAME, O_CREAT, 0666, 1);
   if (SEM_FAILED == shm_sem)
   {
      LOG_ERR("sem_open failed");
      return -1;
   }

   LOG_INF("Shared memory initialized");
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
 * \details Unmaps and unlinks shared memory, closes and unlinks semaphore,
 *          closes SQLite database, and closes log file.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void cleanup(void)
{
   if (NULL != shm_data)
   {
      (void)munmap(shm_data, sizeof(struct SharedSensorData));
      (void)shm_unlink(SHM_NAME);
   }

   if (NULL != shm_sem)
   {
      (void)sem_close(shm_sem);
      (void)sem_unlink(SEM_NAME);
   }

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
 *          memory, database, and sensor pipe, then spawns worker threads.
 *          Joins all threads before cleanup and exit.
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
