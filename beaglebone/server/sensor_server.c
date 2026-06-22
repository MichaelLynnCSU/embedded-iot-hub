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
 *
 * \note    Log taxonomy (2026-06-15):
 *          sensor_server.c emits [SERVER] lines only for orchestration
 *          events (start, shutdown). It never logs data-stage events;
 *          those belong to [UART], [PARSE], and [PIPE] respectively.
 *          See each module's file header for its event taxonomy.
 *
 * \note    Log split (2026-06-21):
 *          Two additional log files added alongside sensor_server.log:
 *            - telemetry.log  -- one line per frame, numeric/boolean state
 *                                only, no strings, no semantic decoding.
 *                                High-volume; rotate aggressively.
 *            - events.log     -- one line per events[] entry from hub delta
 *                                gate. Low-volume; audit trail.
 *          log_telemetry() and log_event() write to these files.
 *          log_msg() is unchanged and continues to serve [SERVER], [UART],
 *          [PARSE], and [PIPE] lines in sensor_server.log.
 *          All three writers use the same timestamp call so wall-clock
 *          correlation across files is exact.
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

#define LOG_FILE      "/var/log/sensor_server.log" /**< operational log        */
#define TEL_LOG_FILE  "/var/log/telemetry.log"     /**< numeric state snapshots */
#define EVT_LOG_FILE  "/var/log/events.log"        /**< hub event ledger        */
#define READ_SLEEP_US 10000                        /**< main loop sleep us      */
#define PROCESS_BUF   4096                         /**< stable frame copy buffer */

static volatile int  g_running  = 1;    /**< main loop run flag          */
static FILE         *g_log_fp   = NULL; /**< sensor_server.log handle    */
static FILE         *g_tel_fp   = NULL; /**< telemetry.log handle        */
static FILE         *g_evt_fp   = NULL; /**< events.log handle           */

/******************************************************************************
 * \brief Initialize all three log files with line buffering.
 *
 * \return void
 ******************************************************************************/
void log_init(void)
{
   g_log_fp = fopen(LOG_FILE, "a");
   if (NULL == g_log_fp)
   {
      perror("log fopen");
   }
   else
   {
      (void)setvbuf(g_log_fp, NULL, _IOLBF, 0);
   }

   g_tel_fp = fopen(TEL_LOG_FILE, "a");
   if (NULL == g_tel_fp)
   {
      perror("telemetry log fopen");
   }
   else
   {
      (void)setvbuf(g_tel_fp, NULL, _IOLBF, 0);
   }

   g_evt_fp = fopen(EVT_LOG_FILE, "a");
   if (NULL == g_evt_fp)
   {
      perror("events log fopen");
   }
   else
   {
      (void)setvbuf(g_evt_fp, NULL, _IOLBF, 0);
   }
}

/******************************************************************************
 * \brief Write timestamped message to stdout and sensor_server.log.
 *
 * \details Used for [SERVER], [UART], [PARSE], [PIPE] operational lines.
 *          Not used for telemetry or event log lines.
 *
 * \param p_fmt - printf-style format string.
 * \param ...   - Format arguments.
 ******************************************************************************/
void log_msg(const char *p_fmt, ...)
{
   time_t    now  = 0;
   struct tm *p_t = NULL;
   va_list   args;

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
 * \brief Write one telemetry snapshot line to telemetry.log.
 *
 * \details Numeric and boolean device state only — no strings, no semantic
 *          decoding. Exactly one call per frame from json_parser.c.
 *          Format is fixed so awk field positions never shift.
 *
 * \param p_fmt - printf-style format string.
 * \param ...   - Format arguments.
 ******************************************************************************/
void log_telemetry(const char *p_fmt, ...)
{
   time_t    now  = 0;
   struct tm *p_t = NULL;
   va_list   args;

   now = time(NULL);
   p_t = localtime(&now);

   if (NULL != g_tel_fp)
   {
      fprintf(g_tel_fp, "[%02d:%02d:%02d] ",
              p_t->tm_hour, p_t->tm_min, p_t->tm_sec);
      va_start(args, p_fmt);
      (void)vfprintf(g_tel_fp, p_fmt, args);
      va_end(args);
      fprintf(g_tel_fp, "\n");
      (void)fflush(g_tel_fp);
   }
}

/******************************************************************************
 * \brief Write one event line to events.log.
 *
 * \details One call per events[] entry from the hub delta gate.
 *          Low-volume; every line represents a true state transition.
 *          frame_seq is the join key to correlate with telemetry.log.
 *
 * \param p_fmt - printf-style format string.
 * \param ...   - Format arguments.
 ******************************************************************************/
void log_event(const char *p_fmt, ...)
{
   time_t    now  = 0;
   struct tm *p_t = NULL;
   va_list   args;

   now = time(NULL);
   p_t = localtime(&now);

   if (NULL != g_evt_fp)
   {
      fprintf(g_evt_fp, "[%02d:%02d:%02d] ",
              p_t->tm_hour, p_t->tm_min, p_t->tm_sec);
      va_start(args, p_fmt);
      (void)vfprintf(g_evt_fp, p_fmt, args);
      va_end(args);
      fprintf(g_evt_fp, "\n");
      (void)fflush(g_evt_fp);
   }
}

/******************************************************************************
 * \brief POSIX signal handler — initiates graceful shutdown.
 *
 * \param sig - Signal number received.
 ******************************************************************************/
static void signal_handler(int sig)
{
   (void)sig;

   log_msg("[SERVER] shutdown");
   g_running = 0;

   uart_io_close();
   pipe_writer_close();

   if (NULL != g_log_fp) { fclose(g_log_fp); }
   if (NULL != g_tel_fp) { fclose(g_tel_fp); }
   if (NULL != g_evt_fp) { fclose(g_evt_fp); }
}

/******************************************************************************
 * \brief Application entry point.
 *
 * \return int - 0 on clean exit, 1 on UART init failure.
 ******************************************************************************/
int main(void)
{
   char g_process[PROCESS_BUF];

   log_init();
   log_msg("=========================================");
   log_msg("[SERVER] start");
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

      while (uart_io_next_frame(g_process, PROCESS_BUF))
      {
         process_json(g_process);
      }

      usleep(READ_SLEEP_US);
   }

   log_msg("[SERVER] shutdown complete");
   return 0;
}
