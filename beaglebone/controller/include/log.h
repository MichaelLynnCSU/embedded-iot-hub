/******************************************************************************
 * \file log.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-04-29
 *
 * \brief Thread-safe timestamped logging for BeagleBone data controller.
 *
 * \details Provides LOG_ERR, LOG_WRN, LOG_INF, LOG_DBG macros and the
 *          LOG backward-compatible alias. All writes are serialised by
 *          log_mutex. Timestamps use localtime_r() — thread-safe.
 *          Log file is rotated to .old at LOG_MAX_BYTES (5 MB).
 *          LOG_DBG compiles out when NDEBUG is defined.
 ******************************************************************************/
#ifndef INCLUDE_LOG_H_
#define INCLUDE_LOG_H_

#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include "config.h"

extern FILE            *log_fp;     /**< opened by data_controller.c */
extern pthread_mutex_t  log_mutex;  /**< serialises all log writes   */

/**
 * \brief Rotate log if it exceeds LOG_MAX_BYTES. Call with log_mutex held.
 */
static inline void log_rotate_if_needed(void)
{
   long pos = 0;
   if (NULL == log_fp) { return; }
   pos = ftell(log_fp);
   if (pos < LOG_MAX_BYTES) { return; }
   fclose(log_fp);
   (void)rename(CONTROLLER_LOG, CONTROLLER_LOG_OLD);
   log_fp = fopen(CONTROLLER_LOG, "a");
}

#define _LOG_CORE(level, fmt, ...) \
do \
{ \
   pthread_mutex_lock(&log_mutex); \
   if (log_fp) \
   { \
      time_t      _t  = time(NULL); \
      struct tm   _tm; \
      (void)localtime_r(&_t, &_tm); \
      log_rotate_if_needed(); \
      fprintf(log_fp, \
              "[%04d-%02d-%02d %02d:%02d:%02d] [%-3s] [tid=%08lx] " fmt "\n", \
              _tm.tm_year + 1900, _tm.tm_mon + 1, _tm.tm_mday, \
              _tm.tm_hour, _tm.tm_min, _tm.tm_sec, \
              (level), \
              (unsigned long)pthread_self(), \
              ##__VA_ARGS__); \
      fflush(log_fp); \
   } \
   pthread_mutex_unlock(&log_mutex); \
} while (0)

#define LOG_ERR(fmt, ...)  _LOG_CORE("ERR", fmt, ##__VA_ARGS__)
#define LOG_WRN(fmt, ...)  _LOG_CORE("WRN", fmt, ##__VA_ARGS__)
#define LOG_INF(fmt, ...)  _LOG_CORE("INF", fmt, ##__VA_ARGS__)
#ifndef NDEBUG
#define LOG_DBG(fmt, ...)  _LOG_CORE("DBG", fmt, ##__VA_ARGS__)
#else
#define LOG_DBG(fmt, ...)  do {} while (0)
#endif
#define LOG(fmt, ...)      LOG_INF(fmt, ##__VA_ARGS__)

#endif /* INCLUDE_LOG_H_ */
