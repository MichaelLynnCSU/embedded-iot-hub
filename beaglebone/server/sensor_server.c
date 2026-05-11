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
 *
 * \note    Double buffer refactor (2026-05-10):
 *
 *          PROBLEM:
 *          The original design used a single linear accumulation buffer:
 *
 *            char buffer[BUFFER_SIZE];
 *            int  pos;
 *
 *          After extracting a JSON frame, the buffer was wiped (pos = 0).
 *          If process_json() blocked (pipe stall, slow logging) while the
 *          ESP32 sent a second frame, the second frame arrived into the
 *          same buffer before pos was reset. The parser cleared everything
 *          after frame A, silently dropping frame B.
 *
 *          The frame-drop scenario:
 *            1. ESP sends frame A
 *            2. process_json() blocks on pipe write or log I/O
 *            3. ESP sends frame B -- bytes land in buffer on top of A
 *            4. pos resets after A is processed
 *            5. frame B bytes are gone
 *
 *          PRINCIPLE:
 *          Always copy data into a stable local buffer before processing.
 *          Never work directly on a buffer the producer can still write
 *          into. This is the same pattern used throughout the controller:
 *          lock, copy, unlock, then work on the copy.
 *
 *          FIX:
 *          uart_io.c owns the active accumulation buffer. When a complete
 *          frame is found, uart_io_next_frame() copies it into the caller's
 *          stable process buffer and compacts the active buffer with memmove
 *          to preserve any remaining bytes. process_json() always receives
 *          the stable copy. Back-to-back frames are drained in a loop.
 *
 *          Deployment assumptions (intentional, document here):
 *            - One ESP32 producer, fixed telemetry cadence
 *            - Dedicated UART, no ISR-driven ingestion
 *            - Linux userspace, no hard realtime requirement
 *            - process_json() expected to be fast under normal conditions
 *            - UART backlog expected to remain small
 *
 *          If any of these assumptions change (multiple producers, faster
 *          publish rates, binary framing, sustained backlog) a ring buffer
 *          feeding a consumer thread is the appropriate next step.
 *
 * \note    Module split (2026-05-10):
 *          sensor_server.c was split into four files as it grew:
 *            - uart_io.c      -- UART init, accumulation buffer, frame extraction
 *            - json_parser.c  -- JSON parsing, SensorData population
 *            - pipe_writer.c  -- named pipe open, reconnect, write
 *            - sensor_server.c -- main(), signal handler, log, wiring
 *          sensor_types.h holds shared wire format structs (SensorData,
 *          ReedSlotData). server_log.h exposes log_msg() to all modules.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <stdarg.h>
#include "uart_io.h"
#include "json_parser.h"
#include "pipe_writer.h"
#include "server_log.h"

#define LOG_FILE      "/var/log/sensor_server.log" /**< log file path */
#define READ_SLEEP_US 10000                        /**< main loop sleep us */
#define PROCESS_BUF   4096                         /**< stable frame copy buffer */

static volatile int  g_running = 1;    /**< main loop run flag */
static FILE         *g_log_fp  = NULL; /**< log file handle -- owned here */

/******************************************************************************
 * \brief Initialize log file with line buffering.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void log_init(void)
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
void log_msg(const char *p_fmt, ...)
{
   time_t    now  = 0;    /**< current timestamp */
   struct tm *p_t = NULL; /**< broken-down time */
   va_list   args;        /**< variadic argument list */

   now = time(NULL);
   p_t = localtime(&now);

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

   uart_io_close();
   pipe_writer_close();

   if (NULL != g_log_fp)
   {
      fclose(g_log_fp);
   }
}

/******************************************************************************
 * \brief Application entry point.
 *
 * \return int - 0 on clean exit, 1 on UART init failure.
 *
 * \details Initializes logging, UART, and pipe. Main loop reads bytes via
 *          uart_io_read(), then drains all complete frames with
 *          uart_io_next_frame() into g_process[] before calling
 *          process_json(). The loop handles back-to-back frames naturally.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int main(void)
{
   /* Stable frame copy -- process_json() always works on this buffer,
    * never on the active UART accumulation buffer owned by uart_io.c */
   char g_process[PROCESS_BUF];

   log_init();
   log_msg("=========================================");
   log_msg("Sensor Server Starting");
   log_msg("=========================================");

   (void)signal(SIGINT,  signal_handler);
   (void)signal(SIGTERM, signal_handler);

   if (0 > uart_io_init())
   {
      return 1;
   }

   pipe_writer_try_open();
   log_msg("Controller pipe: will retry automatically if not ready");
   log_msg("Waiting for data...");

   while (g_running)
   {
      uart_io_read();

      /* Drain all complete frames found in the active buffer.
       * uart_io_next_frame() copies into g_process[] and compacts
       * the active buffer -- back-to-back frames are preserved. */
      while (uart_io_next_frame(g_process, PROCESS_BUF))
      {
         process_json(g_process);
      }

      usleep(READ_SLEEP_US);
   }

   log_msg("Server exiting");
   return 0;
}
