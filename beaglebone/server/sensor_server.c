/******************************************************************************
 * \file sensor_server.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief BeagleBone sensor server — receives ESP32 JSON and forwards to
 *        data controller via named pipe.
 *
 * \details Reads raw bytes from ESP32 hub over UART (ttyS4 at 115200).
 *          Extracts complete JSON objects from the byte stream, parses
 *          them into SensorData structs, and writes to the sensor pipe
 *          for the data controller to consume.
 *
 * \warning SensorData and ReedSlotData structs must be byte-for-byte
 *          identical with controller_internal.h. Both files must be
 *          updated together if the layout changes.
 *
 *          Reed sensors appear automatically as ESP32 discovers them.
 *          offline=1 means device age > 150s (red dot, tile stays visible).
 *          gen increments when a slot is reassigned to a different device.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <json-c/json.h>
#include <time.h>
#include <stdarg.h>
#include <stdint.h>

#define UART_PORT        "/dev/ttyS4"          /**< ESP32 hub UART device */
#define BAUDRATE         B115200               /**< UART baud rate */
#define SENSOR_PIPE      "/tmp/sensor_pipe"    /**< named pipe to controller */
#define MAX_ROOMS        10                    /**< max room sensors per frame */
#define MAX_REEDS        6                     /**< must match ESP32 and controller */
#define BUFFER_SIZE      4096                  /**< UART receive buffer size */
#define LOG_FILE         "/var/log/sensor_server.log" /**< log file path */
#define PIPE_RETRY_SEC   3                     /**< pipe reconnect interval s */
#define READ_BUF_SIZE    256                   /**< UART read chunk size */
#define READ_SLEEP_US    10000                 /**< main loop sleep us */
#define AGE_MAX          0xFFFE               /**< maximum reportable age */
#define AGE_UNKNOWN      0xFFFF              /**< age sentinel: never seen */
#define REED_NAME_LEN    16                   /**< reed BLE name buffer size */
#define ROOM_NAME_LEN    32                   /**< room name buffer size */
#define ROOM_STATE_LEN   16                   /**< room state buffer size */
#define ROOM_LOC_LEN     32                   /**< room location buffer size */

static volatile int  g_running       = 1;    /**< main loop run flag */
static int           g_uart_fd       = -1;   /**< UART file descriptor */
static int           g_pipe_fd       = -1;   /**< pipe file descriptor */
static FILE         *g_log_fp        = NULL; /**< log file handle */
static time_t        g_last_pipe_retry = 0;  /**< last pipe retry timestamp */

/************************ STRUCTURE/UNION DATA TYPES **************************/

/**
 * \brief Reed sensor slot — packed wire format.
 *
 * \warning Must match controller_internal.h ReedSlotData exactly.
 */
struct __attribute__((packed)) ReedSlotData
{
   uint16_t age;             /*!< seconds since last adv, 0xFFFF=never */
   int8_t   batt;            /*!< battery SOC percent, -1=unknown */
   uint8_t  active;          /*!< 1=slot occupied (ACTIVE or OFFLINE) */
   uint8_t  state;           /*!< 0=closed 1=open 0xFF=unknown */
   uint8_t  offline;         /*!< 1=SLOT_OFFLINE, red dot, tile stays */
   uint16_t gen;             /*!< generation counter, increments on swap */
   char     name[REED_NAME_LEN]; /*!< BLE advertised name */
};

/**
 * \brief Sensor data pipe wire format.
 *
 * \warning Must match controller_internal.h SensorData exactly.
 */
struct SensorData
{
   double   avg_temp;        /*!< average temperature in Celsius */
   int      motion_count;    /*!< PIR motion event count */
   int      light_state;     /*!< smart light relay state */
   int      lock_state;      /*!< smart lock state */
   long     timestamp;       /*!< Unix timestamp of reading */
   int      room_count;      /*!< number of valid room entries */

   struct
   {
      int  sensor_id;                /*!< room sensor identifier */
      char room_name[ROOM_NAME_LEN]; /*!< room name string */
      char state[ROOM_STATE_LEN];    /*!< current state string */
      char location[ROOM_LOC_LEN];   /*!< physical location string */
   } rooms[MAX_ROOMS];

   uint16_t age_pir;         /*!< PIR device age seconds */
   uint16_t age_lgt;         /*!< light device age seconds */
   uint16_t age_lck;         /*!< lock device age seconds */
   int8_t   batt_pir;        /*!< PIR battery SOC percent */
   int8_t   batt_lck;        /*!< lock battery SOC percent */

   struct ReedSlotData reed_slots[MAX_REEDS]; /*!< dynamic reed slot array */
   uint8_t  motor_online;  /*!< 1 if C3 motor controller is online */
};

/******************************************************************************
 * \brief Initialize log file with line buffering.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void log_init(void)
{
   g_log_fp = fopen(LOG_FILE, "a");
   if (NULL == g_log_fp)
   {
      perror("log fopen");
      return;
   }

   (void)setvbuf(g_log_fp, NULL, _IOLBF, 0);
}

/******************************************************************************
 * \brief Write timestamped message to stdout and log file.
 *
 * \param p_fmt - printf-style format string.
 * \param ...   - Format arguments.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void log_msg(const char *p_fmt, ...)
{
   time_t    now  = 0;    /**< current timestamp */
   struct tm *p_t = NULL; /**< broken-down time */
   va_list   args;        /**< variadic argument list */

   now  = time(NULL);
   p_t  = localtime(&now);

   printf("[%02d:%02d:%02d] ", p_t->tm_hour, p_t->tm_min, p_t->tm_sec);
   va_start(args, p_fmt);
   (void)vprintf(p_fmt, args);
   va_end(args);
   printf("\n");
   (void)fflush(stdout);

   if (NULL != g_log_fp)
   {
      fprintf(g_log_fp, "[%02d:%02d:%02d] ",
              p_t->tm_hour, p_t->tm_min, p_t->tm_sec);
      va_start(args, p_fmt);
      (void)vfprintf(g_log_fp, p_fmt, args);
      va_end(args);
      fprintf(g_log_fp, "\n");
      (void)fflush(g_log_fp);
   }
}

/******************************************************************************
 * \brief POSIX signal handler — initiates graceful shutdown.
 *
 * \param sig - Signal number received.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void signal_handler(int sig)
{
   (void)sig;

   log_msg("Shutdown signal received");
   g_running = 0;

   if (0 <= g_uart_fd)
   {
      close(g_uart_fd);
   }

   if (0 <= g_pipe_fd)
   {
      close(g_pipe_fd);
   }

   if (NULL != g_log_fp)
   {
      fclose(g_log_fp);
   }
}

/******************************************************************************
 * \brief Initialize UART at 115200 8N1.
 *
 * \return int - 0 on success, -1 on failure.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static int init_uart(void)
{
   struct termios tty; /**< terminal settings */

   g_uart_fd = open(UART_PORT, O_RDWR | O_NOCTTY);
   if (0 > g_uart_fd)
   {
      log_msg("ERROR opening UART: %s", strerror(errno));
      return -1;
   }

   (void)tcgetattr(g_uart_fd, &tty);
   (void)cfsetospeed(&tty, BAUDRATE);
   (void)cfsetispeed(&tty, BAUDRATE);

   tty.c_cflag &= ~PARENB;
   tty.c_cflag &= ~CSTOPB;
   tty.c_cflag &= ~CSIZE;
   tty.c_cflag |=  CS8;
   tty.c_cflag &= ~CRTSCTS;
   tty.c_cflag |=  CREAD | CLOCAL;
   tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
   tty.c_iflag &= ~(IXON | IXOFF | IXANY);
   tty.c_oflag &= ~OPOST;
   tty.c_cc[VTIME] = 10;
   tty.c_cc[VMIN]  = 0;

   (void)tcsetattr(g_uart_fd, TCSANOW, &tty);
   (void)tcflush(g_uart_fd, TCIOFLUSH);

   log_msg("UART initialized on %s @115200", UART_PORT);
   return 0;
}

/******************************************************************************
 * \brief Attempt to open the sensor named pipe in non-blocking write mode.
 *
 * \return int - File descriptor on success, -1 if pipe not ready.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static int pipe_try_open(void)
{
   int fd = -1; /**< pipe file descriptor */

   fd = open(SENSOR_PIPE, O_WRONLY | O_NONBLOCK);
   if (0 <= fd)
   {
      log_msg("Connected to controller pipe");
   }

   return fd;
}

/******************************************************************************
 * \brief Write a SensorData struct to the named pipe.
 *
 * \param p_data - Pointer to SensorData struct to write.
 *
 * \return void
 *
 * \details Closes and resets g_pipe_fd on write failure.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void pipe_write(struct SensorData *p_data)
{
   ssize_t w = 0; /**< bytes written */

   if (0 > g_pipe_fd)
   {
      return;
   }

   w = write(g_pipe_fd, p_data, sizeof(*p_data));
   if (w != (ssize_t)sizeof(*p_data))
   {
      log_msg("Pipe write failed (%zd), will reconnect", w);
      close(g_pipe_fd);
      g_pipe_fd = -1;
   }
   else
   {
      log_msg("Sent to controller");
   }
}

/******************************************************************************
 * \brief Ensure pipe is connected, retrying at PIPE_RETRY_SEC intervals.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void pipe_ensure_connected(void)
{
   time_t now = 0; /**< current timestamp */

   if (0 <= g_pipe_fd)
   {
      return;
   }

   now = time(NULL);
   if ((now - g_last_pipe_retry) < PIPE_RETRY_SEC)
   {
      return;
   }

   g_last_pipe_retry = now;
   g_pipe_fd = pipe_try_open();

   if (0 > g_pipe_fd)
   {
      log_msg("Controller pipe not available, retrying in %ds",
              PIPE_RETRY_SEC);
   }
}

/******************************************************************************
 * \brief Extract age value from JSON object with range clamping.
 *
 * \param p_root - JSON root object.
 * \param p_key  - Key string to look up.
 *
 * \return uint16_t - Age in seconds, AGE_UNKNOWN if missing or negative,
 *                    clamped to AGE_MAX if too large.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static uint16_t json_get_age(struct json_object *p_root, const char *p_key)
{
   struct json_object *p_obj = NULL; /**< extracted JSON object */
   int                 v     = 0;    /**< integer value */

   if (!json_object_object_get_ex(p_root, p_key, &p_obj))
   {
      return AGE_UNKNOWN;
   }

   v = json_object_get_int(p_obj);

   if (0 > v)
   {
      return AGE_UNKNOWN;
   }

   if (v > (int)AGE_MAX)
   {
      return (uint16_t)AGE_MAX;
   }

   return (uint16_t)v;
}

/******************************************************************************
 * \brief Parse reed slots from JSON reeds array into SensorData.
 *
 * \param p_reeds_arr - JSON array object for "reeds" key.
 * \param p_data      - Pointer to SensorData to populate.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void parse_reed_slots(struct json_object *p_reeds_arr,
                              struct SensorData  *p_data)
{
   int                 n      = 0;    /**< array length */
   int                 i      = 0;    /**< loop index */
   int                 id     = 0;    /**< reed slot id */
   int                 slot   = 0;    /**< slot index */
   int                 age    = 0;    /**< raw age value */
   struct json_object *p_r    = NULL; /**< single reed entry */
   struct json_object *p_jid  = NULL; /**< id field */
   struct json_object *p_jval = NULL; /**< generic value field */

   n = json_object_array_length(p_reeds_arr);

   for (i = 0; i < n; i++)
   {
      p_r = json_object_array_get_idx(p_reeds_arr, i);

      if (!json_object_object_get_ex(p_r, "id", &p_jid))
      {
         continue;
      }

      id   = json_object_get_int(p_jid);
      slot = id - 1;

      if ((0 > slot) || (slot >= MAX_REEDS))
      {
         continue;
      }

      p_data->reed_slots[slot].active = 1;

      if (json_object_object_get_ex(p_r, "batt", &p_jval))
      {
         p_data->reed_slots[slot].batt =
            (int8_t)json_object_get_int(p_jval);
      }

      if (json_object_object_get_ex(p_r, "age", &p_jval))
      {
         age = json_object_get_int(p_jval);
         p_data->reed_slots[slot].age =
            (0 > age) ? AGE_UNKNOWN : (uint16_t)age;
      }

      if (json_object_object_get_ex(p_r, "state", &p_jval))
      {
         p_data->reed_slots[slot].state =
            (uint8_t)json_object_get_int(p_jval);
      }

      if (json_object_object_get_ex(p_r, "offline", &p_jval))
      {
         p_data->reed_slots[slot].offline =
            (uint8_t)json_object_get_int(p_jval);
      }

      if (json_object_object_get_ex(p_r, "gen", &p_jval))
      {
         p_data->reed_slots[slot].gen =
            (uint16_t)json_object_get_int(p_jval);
      }

      if (json_object_object_get_ex(p_r, "name", &p_jval))
      {
         (void)strncpy(p_data->reed_slots[slot].name,
                       json_object_get_string(p_jval),
                       REED_NAME_LEN - 1);
      }

      log_msg("  Reed slot %d (%s): state=%s batt=%d%% age=%d "
              "offline=%d gen=%u",
              slot + 1,
              p_data->reed_slots[slot].name,
              (1 == p_data->reed_slots[slot].state) ? "open"  :
              (0 == p_data->reed_slots[slot].state) ? "closed": "unknown",
              p_data->reed_slots[slot].batt,
              p_data->reed_slots[slot].age,
              p_data->reed_slots[slot].offline,
              p_data->reed_slots[slot].gen);
   }
}

/******************************************************************************
 * \brief Parse room entries from JSON rooms array into SensorData.
 *
 * \param p_rooms_arr - JSON array object for "rooms" key.
 * \param p_data      - Pointer to SensorData to populate.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void parse_rooms(struct json_object *p_rooms_arr,
                         struct SensorData  *p_data)
{
   int                 n      = 0;    /**< array length */
   int                 i      = 0;    /**< loop index */
   struct json_object *p_r    = NULL; /**< single room entry */
   struct json_object *p_jval = NULL; /**< generic value field */

   n = json_object_array_length(p_rooms_arr);

   for (i = 0; (i < n) && (p_data->room_count < MAX_ROOMS); i++)
   {
      p_r = json_object_array_get_idx(p_rooms_arr, i);

      if (!json_object_object_get_ex(p_r, "sensor_id", &p_jval))
      {
         continue;
      }

      p_data->rooms[p_data->room_count].sensor_id =
         json_object_get_int(p_jval);

      if (json_object_object_get_ex(p_r, "room", &p_jval))
      {
         (void)strncpy(p_data->rooms[p_data->room_count].room_name,
                       json_object_get_string(p_jval),
                       ROOM_NAME_LEN - 1);
      }

      if (json_object_object_get_ex(p_r, "state", &p_jval))
      {
         (void)strncpy(p_data->rooms[p_data->room_count].state,
                       json_object_get_string(p_jval),
                       ROOM_STATE_LEN - 1);
      }

      if (json_object_object_get_ex(p_r, "location", &p_jval))
      {
         (void)strncpy(p_data->rooms[p_data->room_count].location,
                       json_object_get_string(p_jval),
                       ROOM_LOC_LEN - 1);
      }

      log_msg("  Room id=%d %s/%s = %s",
              p_data->rooms[p_data->room_count].sensor_id,
              p_data->rooms[p_data->room_count].room_name,
              p_data->rooms[p_data->room_count].location,
              p_data->rooms[p_data->room_count].state);

      p_data->room_count++;
   }
}

/******************************************************************************
 * \brief Parse a JSON body string into SensorData and write to pipe.
 *
 * \param p_json_body - Null-terminated JSON string to parse.
 *
 * \return void
 *
 * \details Initialises SensorData with sentinel values, extracts all
 *          fields, delegates reed and room parsing to helpers, then
 *          writes to the named pipe.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void process_json(const char *p_json_body)
{
   struct json_object *p_root = NULL; /**< parsed JSON root */
   struct json_object *p_obj  = NULL; /**< generic field object */
   struct SensorData   data;          /**< output sensor data struct */
   int                 i      = 0;    /**< loop index */

   p_root = json_tokener_parse(p_json_body);
   if (NULL == p_root)
   {
      log_msg("Invalid JSON");
      return;
   }

   (void)memset(&data, 0, sizeof(data));
   data.timestamp = time(NULL);
   data.age_pir   = AGE_UNKNOWN;
   data.age_lgt   = AGE_UNKNOWN;
   data.age_lck   = AGE_UNKNOWN;
   data.batt_pir  = -1;
   data.batt_lck  = -1;

   for (i = 0; i < MAX_REEDS; i++)
   {
      data.reed_slots[i].age     = AGE_UNKNOWN;
      data.reed_slots[i].batt    = -1;
      data.reed_slots[i].active  = 0;
      data.reed_slots[i].state   = 0xFF;
      data.reed_slots[i].offline = 0;
      data.reed_slots[i].gen     = 0;
   }

   if (json_object_object_get_ex(p_root, "avg_temp", &p_obj))
   {
      data.avg_temp = json_object_get_double(p_obj);
   }

   if (json_object_object_get_ex(p_root, "motion_count", &p_obj))
   {
      data.motion_count = json_object_get_int(p_obj);
   }

   if (json_object_object_get_ex(p_root, "light_state", &p_obj))
   {
      data.light_state = json_object_get_int(p_obj);
   }

   if (json_object_object_get_ex(p_root, "lock_state", &p_obj))
   {
      data.lock_state = json_object_get_int(p_obj);
   }

   data.age_pir = json_get_age(p_root, "age_pir");
   data.age_lgt = json_get_age(p_root, "age_lgt");
   data.age_lck = json_get_age(p_root, "age_lck");

   if (json_object_object_get_ex(p_root, "batt_pir", &p_obj))
   {
      data.batt_pir = (int8_t)json_object_get_int(p_obj);
   }

   if (json_object_object_get_ex(p_root, "batt_lck", &p_obj))
   {
      data.batt_lck = (int8_t)json_object_get_int(p_obj);
   }

   if (json_object_object_get_ex(p_root, "motor_online", &p_obj))
   {
      data.motor_online = (uint8_t)json_object_get_int(p_obj);
      log_msg("motor_online=%d", data.motor_online);
   }

   if (json_object_object_get_ex(p_root, "reeds", &p_obj))
   {
      parse_reed_slots(p_obj, &data);
   }

   log_msg("Parsed avg_temp=%.2f motion=%d light=%d lock=%d",
           data.avg_temp, data.motion_count,
           data.light_state, data.lock_state);
   log_msg("Ages pir=%d lgt=%d lck=%d | Batt pir=%d%% lck=%d%%",
           data.age_pir, data.age_lgt, data.age_lck,
           data.batt_pir, data.batt_lck);

   if (json_object_object_get_ex(p_root, "rooms", &p_obj))
   {
      parse_rooms(p_obj, &data);
   }

   json_object_put(p_root);

   pipe_ensure_connected();
   pipe_write(&data);
}

/******************************************************************************
 * \brief Extract the first complete JSON object from a byte buffer.
 *
 * \param p_buf - Null-terminated buffer to search.
 *
 * \return char* - Pointer to start of JSON object within p_buf,
 *                 null-terminated after closing brace. NULL if not found.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static char *extract_json(char *p_buf)
{
   char *p_start = NULL; /**< pointer to opening brace */
   char *p       = NULL; /**< scan pointer */
   int   depth   = 0;    /**< brace nesting depth */

   p_start = strchr(p_buf, '{');
   if (NULL == p_start)
   {
      return NULL;
   }

   for (p = p_start; '\0' != *p; p++)
   {
      if ('{' == *p)
      {
         depth++;
      }
      else if ('}' == *p)
      {
         depth--;
         if (0 == depth)
         {
            *(p + 1) = '\0';
            return p_start;
         }
      }
      else
      {
         /* non-brace character — continue */
      }
   }

   return NULL;
}

/******************************************************************************
 * \brief Application entry point.
 *
 * \return int - 0 on clean exit, 1 on UART init failure.
 *
 * \details Initializes logging, UART, and pipe. Reads bytes in a loop,
 *          accumulates into buffer, extracts JSON objects and processes
 *          them. Sleeps READ_SLEEP_US between reads.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int main(void)
{
   char buffer[BUFFER_SIZE] = {0}; /**< UART receive accumulation buffer */
   char tmp[READ_BUF_SIZE]  = {0}; /**< per-read chunk buffer */
   int  pos                 = 0;   /**< current buffer write position */
   int  n                   = 0;   /**< bytes read */
   char *p_json             = NULL; /**< extracted JSON pointer */

   log_init();
   log_msg("=========================================");
   log_msg("Sensor Server Starting");
   log_msg("=========================================");

   (void)signal(SIGINT,  signal_handler);
   (void)signal(SIGTERM, signal_handler);

   if (0 > init_uart())
   {
      return 1;
   }

   g_pipe_fd = pipe_try_open();
   if (0 > g_pipe_fd)
   {
      log_msg("Controller pipe not ready, will retry automatically");
   }

   log_msg("Waiting for data...");

   while (g_running)
   {
      n = (int)read(g_uart_fd, tmp, sizeof(tmp) - 1);

      if (0 < n)
      {
         tmp[n] = 0;

         if ((pos + n) < BUFFER_SIZE)
         {
            (void)memcpy(buffer + pos, tmp, n);
            pos        += n;
            buffer[pos] = 0;

            p_json = extract_json(buffer);
            if (NULL != p_json)
            {
               process_json(p_json);
               pos       = 0;
               buffer[0] = 0;
            }
         }
         else
         {
            log_msg("Buffer overflow, reset");
            pos = 0;
         }
      }

      usleep(READ_SLEEP_US);
   }

   log_msg("Server exiting");
   return 0;
}
