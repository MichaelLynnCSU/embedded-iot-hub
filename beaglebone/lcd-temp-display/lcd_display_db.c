#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <time.h>
#include "lcd_driver.h"
#include "shared_data.h"

#define UPDATE_INTERVAL 2  // seconds
#define LCD_LOG "/var/log/lcd_display.log"

/* ===================== Logging ===================== */

static FILE *log_fp = NULL;

#define LOG(fmt, ...) do {                                  \
    if (log_fp) {                                           \
        time_t _t = time(NULL);                             \
        struct tm *_tm = localtime(&_t);                    \
        fprintf(log_fp,                                    \
            "[%04d-%02d-%02d %02d:%02d:%02d] " fmt "\n",    \
            _tm->tm_year + 1900,                            \
            _tm->tm_mon + 1,                                \
            _tm->tm_mday,                                   \
            _tm->tm_hour,                                   \
            _tm->tm_min,                                    \
            _tm->tm_sec,                                    \
            ##__VA_ARGS__);                                  \
        fflush(log_fp);                                     \
    }                                                       \
} while (0)

/* ===================== Main ===================== */

int main(void) {
    // Open log file first
    log_fp = fopen(LCD_LOG, "a");
    if (!log_fp) {
        perror("log open failed");
        return 1;
    }

    LOG("=========================================");
    LOG("LCD Display starting");
    LOG("=========================================");

    printf("Initializing LCD display...\n");
    LOG("Initializing LCD hardware");

    // 1. Map GPIO banks FIRST
    map_gpio_banks();
    
    // 2. Configure pins as outputs
    for (int i = 0; i < NUM_PINS; i++) {
        gpio_set_output(lcd_pins[i].bank, lcd_pins[i].bit);
    }
    
    // 3. Initialize LCD
    lcd_init();
    LOG("LCD hardware initialized");
    
    // 4. Test display
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Display Up!");
    printf("'Display Up!' should be visible\n");
    LOG("LCD test message displayed");
    sleep(2);

    // Open shared memory (instead of database)
    LOG("Opening shared memory");
    int shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd < 0) {
        LOG("ERROR: Failed to open shared memory");
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("SHM Open Error!");
        perror("shm_open");
        fprintf(stderr, "Make sure data_controller is running!\n");
        return 1;
    }
    
    // Map shared memory
    struct SharedSensorData *shm_data = mmap(NULL,
        sizeof(struct SharedSensorData),
        PROT_READ, MAP_SHARED, shm_fd, 0);
    
    if (shm_data == MAP_FAILED) {
        LOG("ERROR: Failed to map shared memory");
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("SHM Map Error!");
        perror("mmap");
        return 1;
    }
    
    // Open semaphore
    sem_t *shm_sem = sem_open(SEM_NAME, 0);
    if (shm_sem == SEM_FAILED) {
        LOG("ERROR: Failed to open semaphore");
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("SEM Open Error!");
        perror("sem_open");
        return 1;
    }
    
    printf("Connected to shared memory\n");
    LOG("Connected to shared memory successfully");
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Connected!");
    sleep(1);

    double last_temp = -9999;
    int last_motion = -1;
    int last_seq = -1;
    
    LOG("Starting display update loop");

    while (1) {
        // Read from shared memory
        sem_wait(shm_sem);
        double temp = shm_data->current_temp;
        int motion = shm_data->current_motion;
        int valid = shm_data->data_valid;
        int seq = shm_data->sequence;
        sem_post(shm_sem);
        
        // Update LCD only if values changed
        if (valid && (seq != last_seq || temp != last_temp || motion != last_motion)) {
            printf("Updating LCD: temp=%.2f, motion=%d (seq=%d)\n", temp, motion, seq);
            LOG("Display update: temp=%.1fC motion=%d (seq=%d)", temp, motion, seq);
            
            lcd_clear();
            lcd_set_cursor(0, 0);
            char line1[17];
            snprintf(line1, sizeof(line1), "Temp: %.1fC", temp);
            lcd_print(line1);
            
            lcd_set_cursor(1, 0);
            char line2[17];
            snprintf(line2, sizeof(line2), "Motion: %d", motion);
            lcd_print(line2);
            
            last_temp = temp;
            last_motion = motion;
            last_seq = seq;
        } else if (!valid) {
            // No data yet from controller
            if (last_seq == -1) {  // Only show once
                LOG("Waiting for sensor data (data_valid=0)");
                lcd_clear();
                lcd_set_cursor(0, 0);
                lcd_print("Waiting for");
                lcd_set_cursor(1, 0);
                lcd_print("sensor data...");
                last_seq = 0;
            }
        }
        
        sleep(UPDATE_INTERVAL);
    }
    
    // Cleanup
    LOG("Shutting down LCD display");
    munmap(shm_data, sizeof(struct SharedSensorData));
    sem_close(shm_sem);
    close(shm_fd);
    
    if (log_fp) fclose(log_fp);
    
    return 0;
}
