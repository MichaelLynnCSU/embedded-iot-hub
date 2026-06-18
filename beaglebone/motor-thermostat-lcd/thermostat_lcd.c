/******************************************************************************
 * \file thermostat_lcd.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Thermostat LCD display for BeagleBone.
 *
 * \details Reads thermostat state from shared memory and renders it on a
 *          HD44780 16x2 LCD via direct GPIO register access.
 *
 *          This is NOT the smart home dashboard. The smart home dashboard
 *          is the STM32 BlackPill running LVGL on an ILI9341 TFT display.
 *
 *          This LCD shows:
 *            Line 0: average temperature used by the PID loop
 *            Line 1: motor online status and battery percentage
 *
 *          Data source: POSIX shared memory /sensor_shm written by
 *          data_controller. Protected by shm_data->shm_mutex
 *          (PTHREAD_PROCESS_SHARED).
 *
 *          SHM read logged as:
 *            [SHM] transport=sensor_shm event_id=M read dst=thermostat_lcd
 *
 * \note    Semaphore → mutex (2026-05-22):
 *          sem_open/sem_wait/sem_post removed. All shared memory access
 *          now uses pthread_mutex_lock(&shm_data->shm_mutex).
 *
 * \note    Data scope (2026-06-07):
 *          Displays shm_data->current_temp (avg temp for PID) and
 *          shm_data->batt_motor only. Smart home fields (motion, lock,
 *          light, reed, PIR) are not displayed here — those belong on
 *          the STM32 BlackPill dashboard.
 *
 * \note    Renamed from motor_lcd.c (2026-06-18):
 *          motor_lcd implied display of motor state only. This process
 *          owns the full thermostat loop display — temperature sensors,
 *          motor PID, and room cooling state.
 ******************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <time.h>
#include "lcd_driver.h"
#include "../controller/shared_data.h"

/******************************** CONSTANTS ***********************************/

#define SHM_NAME         "/sensor_shm"            /**< shared memory object name   */
#define UPDATE_INTERVAL  2                         /**< display refresh interval s  */
#define LCD_LOG          "/var/log/thermostat_lcd.log" /**< log file path           */

/********************************** LOGGING ***********************************/

static FILE *log_fp = NULL;

#define LOG(fmt, ...) do {                                        \
   if (log_fp) {                                                  \
      time_t    _t  = time(NULL);                                 \
      struct tm _tm;                                              \
      localtime_r(&_t, &_tm);                                     \
      fprintf(log_fp,                                             \
         "[%04d-%02d-%02d %02d:%02d:%02d] " fmt "\n",            \
         _tm.tm_year + 1900, _tm.tm_mon + 1, _tm.tm_mday,        \
         _tm.tm_hour, _tm.tm_min, _tm.tm_sec,                     \
         ##__VA_ARGS__);                                           \
      fflush(log_fp);                                             \
   }                                                              \
} while (0)

/******************************************************************************
 * \brief Program entry point.
 ******************************************************************************/
int main(void)
{
   struct SharedSensorData *shm_data  = NULL;
   int                      shm_fd    = -1;
   double                   last_temp = -9999.0;
   int                      last_batt = -9999;
   int                      last_seq  = -1;

   log_fp = fopen(LCD_LOG, "a");
   if (NULL == log_fp)
   {
      perror("log open failed");
      return 1;
   }

   LOG("=========================================");
   LOG("Thermostat LCD starting");
   LOG("NOT the smart home dashboard (that is the STM32 BlackPill/LVGL)");
   LOG("=========================================");

   /* 1. Map GPIO banks */
   map_gpio_banks();

   /* 2. Configure LCD pins as outputs */
   for (int i = 0; i < NUM_PINS; i++)
   {
      gpio_set_output(lcd_pins[i].bank, lcd_pins[i].bit);
   }

   /* 3. Initialise LCD hardware */
   lcd_init();
   LOG("LCD hardware initialised");

   lcd_clear();
   lcd_set_cursor(0, 0);
   lcd_print("Thermostat");
   lcd_set_cursor(1, 0);
   lcd_print("Starting...");
   sleep(2);

   /* 4. Open shared memory written by data_controller */
   shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
   if (shm_fd < 0)
   {
      LOG("ERROR: shm_open failed — is data_controller running?");
      lcd_clear();
      lcd_set_cursor(0, 0);
      lcd_print("SHM Error!");
      lcd_set_cursor(1, 0);
      lcd_print("No controller");
      perror("shm_open");
      return 1;
   }

   shm_data = mmap(NULL, 3384,
                   PROT_READ|PROT_WRITE, MAP_SHARED, shm_fd, 0);

   if (MAP_FAILED == shm_data)
   {
      LOG("ERROR: mmap failed");
      lcd_clear();
      lcd_set_cursor(0, 0);
      lcd_print("mmap Error!");
      perror("mmap");
      return 1;
   }

   LOG("Connected to shared memory");

   lcd_clear();
   lcd_set_cursor(0, 0);
   lcd_print("SHM OK");
   sleep(1);

   LOG("Starting display update loop");

   while (1)
   {
      double   temp       = 0.0;
      int      batt_motor = -1;
      int      valid      = 0;
      int      seq        = 0;
      int      online     = 0;
      uint64_t event_id   = 0;

      pthread_mutex_lock((pthread_mutex_t *)&shm_data->shm_mutex);
      temp       = shm_data->current_temp;
      batt_motor = shm_data->batt_motor;
      valid      = shm_data->data_valid;
      seq        = shm_data->sequence;
      online     = shm_data->device_online[3]; /* index 3 = DEV_MOTOR */
      event_id   = shm_data->event_id;
      pthread_mutex_unlock((pthread_mutex_t *)&shm_data->shm_mutex);

      LOG("[SHM] transport=sensor_shm event_id=%llu read dst=thermostat_lcd",
          (unsigned long long)event_id);

      if (!valid)
      {
         if (-1 == last_seq)
         {
            LOG("Waiting for data_controller (data_valid=0)");
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("Waiting...");
            lcd_set_cursor(1, 0);
            lcd_print("No data yet");
            last_seq = 0;
         }
      }
      else if (seq != last_seq || temp != last_temp || batt_motor != last_batt)
      {
         char line0[17];
         char line1[17];

         LOG("Thermostat LCD update: temp=%.1fC batt=%d%% online=%d (seq=%d)",
             temp, batt_motor, online, seq);

         /* Line 0: average temperature for PID loop */
         snprintf(line0, sizeof(line0), "PID Tmp:%.1fC", temp);

         /* Line 1: motor online status and battery */
         if (batt_motor >= 0)
         {
            int b = (batt_motor > 100) ? 100 : batt_motor;
            snprintf(line1, sizeof(line1), "Mtr:%s B:%3d%%",
                     online ? "ON " : "OFF", b);
         }
         else
         {
            snprintf(line1, sizeof(line1), "Mtr:%s B:--",
                     online ? "ON " : "OFF");
         }

         lcd_clear();
         lcd_set_cursor(0, 0);
         lcd_print(line0);
         lcd_set_cursor(1, 0);
         lcd_print(line1);

         last_temp = temp;
         last_batt = batt_motor;
         last_seq  = seq;
      }

      sleep(UPDATE_INTERVAL);
   }

   /* Unreachable — service is killed by systemd */
   munmap(shm_data, 3384);
   close(shm_fd);
   if (log_fp) { fclose(log_fp); }

   return 0;
}
