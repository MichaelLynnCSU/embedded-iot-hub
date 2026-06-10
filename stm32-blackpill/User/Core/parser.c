/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    parser.c
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   UART telemetry parser implementation.
 *
 * \details Processes line-oriented ASCII messages from the ESP32. Messages
 *          follow the pattern "ID:field[,field...]". Supported IDs:
 *            REED_COUNT, DR1-DR6, PIR_COUNT, PIR1-PIR4, OCC1-OCC4, PIR,
 *            LGT, LCK, MTR, STATE, OCC, TEMP_COUNT, TEMP1-TEMP4,
 *            DB0-DB3, DOORBELL.
 *          All parsing uses strtol() rather than atoi() for error detection.
 *
 *          Online/offline pattern (all devices including MTR):
 *          - When online: update value and stamp last-seen tick
 *          - When offline: do nothing — HB_TIMEOUT_MS expiry drives dot red
 *          - This is identical behaviour for PIR, LGT, LCK, MTR, and DB
 *
 * \note    OCC frame (2026-04-30):
 *          OCC:0 / OCC:1 sent by BeagleBone every UART_PUSH_INTERVAL_SEC.
 *          Derived from PIR sliding window on ESP32 hub — 1 ISR in the
 *          last 8 s asserts occupied. Routed to ui_set_pir_occupied().
 *          Retained for backward compatibility during rollout.
 *
 * \note    Per-slot PIR frames (2026-05-XX):
 *          PIR_COUNT:n sets the active PIR slot count and triggers
 *          ui_reflow_pir(n) when the count changes — mirrors REED_COUNT.
 *          PIR<n>:count,batt,age updates one PIR slot — mirrors DR<n>.
 *          Both handlers are inserted before the single-value fallthrough
 *          in parser_process_line() so they are dispatched first.
 *
 * \note    Per-slot OCC frames (2026-05-20):
 *          OCC<1-4>:n dispatched before single-value fallthrough, mirrors
 *          PIR<1-4> pattern. Calls ui_set_pir_slot_occupied(slot, val).
 *          Legacy OCC:n handler retained in parse_single_value() for
 *          backward compatibility during rollout.
 *
 * \note    Doorbell liveness (2026-06-09):
 *          DB<0-3>:age_s,online dispatched before DOORBELL: press handler.
 *          parse_doorbell_slot() mirrors parse_pir_slot() / parse_temp_slot().
 *          Stamps ui_stamp_doorbell_online() when online=1.
 *          DB_DIGIT_OFFSET=2, slot is 0-based (- '0' not - '1') matching
 *          hub serialization convention.
 ******************************************************************************/

#include "parser.h"
#include "ui.h"
#include "log.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

/******************************** CONSTANTS ***********************************/

#define BLE_AGE_THRESHOLD_S  300u  /**< Seconds before a BLE device is stale */
#define STATE_FIELD_COUNT      8u  /**< Number of fields in a STATE message   */
#define REED_SLOT_MIN          0   /**< First valid reed slot index            */
#define DR_ID_PREFIX_LEN       2u  /**< Length of "DR" prefix in DRn IDs      */
#define DR_DIGIT_OFFSET        2u  /**< Character offset of digit in "DR1"     */
#define PIR_DIGIT_OFFSET       3u  /**< Character offset of digit in "PIR1"    */
#define OCC_DIGIT_OFFSET       3u  /**< Character offset of digit in "OCC1"    */
#define TEMP_DIGIT_OFFSET      4u  /**< Character offset of digit in "TEMP1"   */
#define DB_DIGIT_OFFSET        2u  /**< Character offset of digit in "DB0"     */

/************************** STATIC (PRIVATE) FUNCTIONS ************************/

/**
 * \brief  Convert an ASCII string to int using strtol with overflow check.
 *
 * \param  p_str    - Null-terminated string to convert.
 * \param  p_result - Output: parsed integer value on success.
 *
 * \return 0 on success, -1 on parse error or overflow.
 */
static int parse_int(const char *p_str, int *p_result)
{
   char   *p_end   = NULL;
   long    val     = 0;

   if ((NULL == p_str) || (NULL == p_result))
   {
      return -1;
   }

   errno = 0;
   val   = strtol(p_str, &p_end, 10);

   if ((errno != 0) || (p_end == p_str))
   {
      return -1;
   }

   *p_result = (int)val;
   return 0;
}

/**
 * \brief  Handle a REED_COUNT:n message — update count and reflow UI.
 *
 * \param  p_rest - String after the colon, containing the count digit.
 */
static void parse_reed_count(const char *p_rest)
{
   int     n           = 0;
   char    log_buf[48] = {0};

   if (0 != parse_int(p_rest, &n))
   {
      return;
   }

   if (n > (int)MAX_REEDS)
   {
      n = (int)MAX_REEDS;
   }

   if ((uint8_t)n != ui_get_reed_count())
   {
      ui_set_reed_count((uint8_t)n);
      ui_reflow(n);
      (void)snprintf(log_buf, sizeof(log_buf), "[UI] Reflow reed_count=%d\r\n", n);
      log_enqueue(log_buf);
   }
}

/**
 * \brief  Handle a PIR_COUNT:n message — update active PIR slot count and
 *         reflow UI.
 *
 * \param  p_rest - String after the colon, containing the count digit.
 *
 * \details Mirrors parse_reed_count(). Only calls ui_reflow_pir() when
 *          the count actually changes to avoid unnecessary LVGL work.
 */
static void parse_pir_count(const char *p_rest)
{
   int     n           = 0;
   char    log_buf[48] = {0};

   if (0 != parse_int(p_rest, &n))
   {
      return;
   }

   if (n > (int)MAX_PIRS) { n = (int)MAX_PIRS; }
   if (n < 0)             { n = 0; }

   if ((uint8_t)n != ui_get_pir_count_slots())
   {
      ui_set_pir_count_slots((uint8_t)n);
      ui_reflow_pir(n);
      (void)snprintf(log_buf, sizeof(log_buf), "[UI] Reflow pir_count=%d\r\n", n);
      log_enqueue(log_buf);
   }
}

static void parse_temp_count(const char *p_rest)
{
   int     n           = 0;
   char    log_buf[48] = {0};

   if (0 != parse_int(p_rest, &n))
   {
      return;
   }

   if (n > (int)MAX_TEMPS) { n = (int)MAX_TEMPS; }
   if (n < 0)              { n = 0; }

   if ((uint8_t)n != ui_get_temp_count_slots())
   {
      ui_set_temp_count_slots((uint8_t)n);
      ui_reflow_temp(n);
      (void)snprintf(log_buf, sizeof(log_buf), "[UI] Reflow temp_count=%d\r\n", n);
      log_enqueue(log_buf);
   }
}

/**
 * \brief  Handle a DRn:state,batt,age message — update one reed sensor slot.
 *
 * \param  slot   - Zero-based reed index (0..MAX_REEDS-1).
 * \param  p_rest - String after the colon: "state,batt,age".
 */
static void parse_reed_sensor(int slot, const char *p_rest)
{
   char    tmp[UART_LINE_LEN] = {0};
   char   *p_tok              = NULL;
   int     state              = -1;
   int     batt               = -1;
   int     age                = 0xFFFF;
   uint32_t now               = 0u;

   if ((slot < REED_SLOT_MIN) || (slot >= (int)MAX_REEDS))
   {
      return;
   }

   if (NULL == p_rest)
   {
      return;
   }

   (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
   tmp[sizeof(tmp) - 1u] = '\0';

   p_tok = strtok(tmp, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &state); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &batt); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &age); }

   now = HAL_GetTick();

   if (state >= 0)
   {
      ui_set_reed_state((uint8_t)slot, (uint8_t)state);
   }

   ui_set_reed_batt((uint8_t)slot, (int8_t)batt);
   ui_set_reed_age((uint8_t)slot,  (uint16_t)age);

   if (age < (int)BLE_AGE_THRESHOLD_S)
   {
      ui_stamp_reed_online((uint8_t)slot, now);
   }
}

/**
 * \brief  Handle a PIR<n>:count,batt,age message — update one PIR slot.
 *
 * \param  slot   - Zero-based PIR index (0..MAX_PIRS-1).
 * \param  p_rest - String after the colon: "count,batt,age".
 *
 * \details Mirrors parse_reed_sensor(). Stamps the slot online when age
 *          is below BLE_AGE_THRESHOLD_S, identical to the reed pattern.
 */
static void parse_pir_slot(int slot, const char *p_rest)
{
   char     tmp[UART_LINE_LEN] = {0};
   char    *p_tok              = NULL;
   int      count              = 0;
   int      batt               = -1;
   int      age                = 0xFFFF;
   uint32_t now                = 0u;

   if ((slot < 0) || (slot >= (int)MAX_PIRS))
   {
      return;
   }

   if (NULL == p_rest)
   {
      return;
   }

   (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
   tmp[sizeof(tmp) - 1u] = '\0';

   p_tok = strtok(tmp, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &count); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &batt); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &age); }

   now = HAL_GetTick();

   ui_set_pir_slot_count((uint8_t)slot, (uint32_t)count);
   ui_set_pir_slot_batt((uint8_t)slot,  (int8_t)batt);
   ui_set_pir_slot_age((uint8_t)slot,   (uint16_t)age);

   if (age < (int)BLE_AGE_THRESHOLD_S)
   {
      ui_stamp_pir_online((uint8_t)slot, now);
   }

   char dbg[96];
   snprintf(dbg, sizeof(dbg),
         "[PIR] slot=%d count=%d batt=%d age=%d\r\n",
         slot, count, batt, age);
   log_enqueue(dbg);
}

static void parse_temp_slot(int slot, const char *p_rest)
{
   char     tmp[UART_LINE_LEN] = {0};
   char    *p_tok              = NULL;
   int      temp_decidegc      = 0;
   int      batt               = -1;
   int      age                = 0xFFFF;
   uint32_t now                = 0u;

   if ((slot < 0) || (slot >= (int)MAX_TEMPS))
   {
      return;
   }

   if (NULL == p_rest)
   {
      return;
   }

   (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
   tmp[sizeof(tmp) - 1u] = '\0';

   p_tok = strtok(tmp, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &temp_decidegc); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &batt); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &age); }

   now = HAL_GetTick();

   ui_set_temp_slot_decidegc((uint8_t)slot, (int16_t)temp_decidegc);
   ui_set_temp_slot_batt((uint8_t)slot,     (int8_t)batt);
   ui_set_temp_slot_age((uint8_t)slot,      (uint16_t)age);

   if (age < (int)BLE_AGE_THRESHOLD_S)
   {
      ui_stamp_temp_online((uint8_t)slot, now);
   }

   char dbg[96];
   snprintf(dbg, sizeof(dbg),
            "[TEMP] slot=%d decidegc=%d batt=%d age=%d\r\n",
            slot, temp_decidegc, batt, age);
   log_enqueue(dbg);
}

/**
 * \brief  Handle a DB<0-3>:age_s,online message — update one doorbell cam slot.
 *
 * \param  slot   - Zero-based doorbell index (0..MAX_DOORBELL_CAMS-1).
 * \param  p_rest - String after the colon: "age_s,online".
 *
 * \details Mirrors parse_temp_slot(). Stamps ui_stamp_doorbell_online()
 *          when online=1. Slot is 0-based — matches hub serialization
 *          (id=0..3, not 1-based like reed/PIR).
 */
static void parse_doorbell_slot(int slot, const char *p_rest)
{
   char     tmp[UART_LINE_LEN] = {0};
   char    *p_tok              = NULL;
   int      age_s              = 0xFFFF;
   int      online             = 0;
   uint32_t now                = 0u;

   if ((slot < 0) || (slot >= (int)MAX_DOORBELL_CAMS))
   {
      return;
   }

   if (NULL == p_rest)
   {
      return;
   }

   (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
   tmp[sizeof(tmp) - 1u] = '\0';

   p_tok = strtok(tmp, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &age_s); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &online); }

   now = HAL_GetTick();

   ui_set_doorbell_slot_age((uint8_t)slot,    (uint16_t)age_s);
   ui_set_doorbell_slot_online((uint8_t)slot, (uint8_t)online);

   if (1 == online)
   {
      ui_stamp_doorbell_online((uint8_t)slot, now);
   }

   char dbg[64];
   snprintf(dbg, sizeof(dbg),
            "[DB] slot=%d age_s=%d online=%d\r\n",
            slot, age_s, online);
   log_enqueue(dbg);
}

/**
 * \brief  Handle a STATE:tmp,pir,lgt,lck,age_pir,age_lgt,age_lck,reed_count
 *         message — bulk sensor update from ESP32.
 *
 * \param  p_rest - String after the colon containing 8 comma-delimited fields.
 */
static void parse_state_message(const char *p_rest)
{
   char    tb[UART_LINE_LEN]         = {0};
   char   *p_tok                     = NULL;
   int     f[STATE_FIELD_COUNT];
   uint8_t i                         = 0u;
   uint32_t now                      = 0u;
   int     old_proto                 = 0;
   int     n                         = 0;

   if (NULL == p_rest)
   {
      return;
   }

   for (i = 0u; i < STATE_FIELD_COUNT; i++)
   {
      f[i] = -1;
   }

   (void)strncpy(tb, p_rest, sizeof(tb) - 1u);
   tb[sizeof(tb) - 1u] = '\0';

   p_tok = strtok(tb, ",");
   for (i = 0u; (i < STATE_FIELD_COUNT) && (NULL != p_tok); i++)
   {
      (void)parse_int(p_tok, &f[i]);
      p_tok = strtok(NULL, ",");
   }

   now = HAL_GetTick();

   if (f[0] >= 0) { ui_set_temp((uint8_t)f[0]);       }
   if (f[1] >= 0) { ui_set_pir_count((uint32_t)f[1]); }
   if (f[2] >= 0) { ui_set_light((uint8_t)f[2]);      }
   if (f[3] >= 0) { ui_set_lock((uint8_t)f[3]);       }

   ui_stamp_dev_online(eDEV_TEMP, now);

   old_proto = (f[4] < 0);

   if ((0 != old_proto) || (f[4] < (int)BLE_AGE_THRESHOLD_S))
   {
      ui_stamp_dev_online(eDEV_PIR, now);
   }

   if ((0 != old_proto) || (f[5] < (int)BLE_AGE_THRESHOLD_S))
   {
      ui_stamp_dev_online(eDEV_LIGHT, now);
   }

   if ((0 != old_proto) || (f[6] < (int)BLE_AGE_THRESHOLD_S))
   {
      ui_stamp_dev_online(eDEV_LOCK, now);
   }

   if (f[7] > 0)
   {
      n = f[7];
      if (n > (int)MAX_REEDS) { n = (int)MAX_REEDS; }

      if ((uint8_t)n != ui_get_reed_count())
      {
         ui_set_reed_count((uint8_t)n);
         ui_reflow(n);
      }
   }
}

/**
 * \brief  Handle a single-value message: PIR, LGT, LCK, MTR, or OCC.
 *
 * \param  p_id  - Null-terminated message identifier string.
 * \param  val   - Primary integer value already parsed from the message.
 * \param  batt  - Battery percentage, or -1 if not present.
 *
 * \note   OCC handler retained here for backward compatibility during rollout.
 *         Per-slot OCC<1-4> is dispatched earlier in parser_process_line().
 */
static void parse_single_value(const char *p_id, int val, int batt)
{
   uint32_t now = HAL_GetTick();

   if (NULL == p_id)
   {
      return;
   }

   if (0 == strcmp(p_id, "PIR"))
   {
      ui_set_pir_count((uint32_t)val);
      if (batt >= 0) { ui_set_pir_batt((uint8_t)batt); }
   }
   else if (0 == strcmp(p_id, "LGT"))
   {
      ui_set_light((uint8_t)val);
   }
   else if (0 == strcmp(p_id, "LCK"))
   {
      ui_set_lock((uint8_t)val);
      if (batt >= 0) { ui_set_lock_batt((int8_t)batt); }
   }
   else if (0 == strcmp(p_id, "MTR"))
   {
      char dbg[48];
      snprintf(dbg, sizeof(dbg), "[MTR] val=%d batt=%d\r\n", val, batt);
      log_enqueue(dbg);
      if (1 == val)
      {
         ui_set_motor(1u);
         ui_stamp_dev_online(eDEV_MOTOR, now);
      }
      if (batt > 0) { ui_set_motor_batt(batt); }
   }
   else if (0 == strcmp(p_id, "OCC"))
   {
      /* Legacy flat OCC — retained for backward compat during rollout */
      ui_set_pir_occupied((uint8_t)val);
   }
   /* Unrecognised single-value ID — discard silently */
}

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  Parse one null-terminated ASCII telemetry line from the ESP32.
 *
 * \param  p_line - Null-terminated message string, e.g. "PIR:42,87".
 *
 * \details Dispatch order:
 *          1. REED_COUNT  — dedicated handler, returns.
 *          2. DR<1-6>     — reed slot handler, returns.
 *          3. PIR_COUNT   — PIR slot count handler, returns.
 *          4. PIR<1-4>    — per-slot PIR handler, returns.
 *          5. OCC<1-4>    — per-slot occupancy handler, returns.
 *          6. TEMP_COUNT  — temp slot count handler, returns.
 *          7. TEMP<1-4>   — per-slot temp handler, returns.
 *          8. DB<0-3>     — per-cam doorbell liveness handler, returns.
 *          9. DOORBELL    — press event handler, returns.
 *         10. STATE       — bulk state handler, returns.
 *         11. Everything else — single-value fallthrough (PIR, LGT, LCK,
 *             MTR, OCC).
 */
void parser_process_line(const char *p_line)
{
   char    buf[UART_LINE_LEN] = {0};
   char   *p_colon            = NULL;
   char   *p_id               = NULL;
   char   *p_rest             = NULL;
   char   *p_comma            = NULL;
   int     val                = 0;
   int     batt               = -1;
   int     slot               = 0;

   if (NULL == p_line)
   {
      return;
   }

   (void)strncpy(buf, p_line, sizeof(buf) - 1u);
   buf[sizeof(buf) - 1u] = '\0';

   p_colon = strchr(buf, ':');
   if (NULL == p_colon)
   {
      return;
   }

   *p_colon = '\0';
   p_id     = buf;
   p_rest   = p_colon + 1;

   /* REED_COUNT:n */
   if (0 == strcmp(p_id, "REED_COUNT"))
   {
      parse_reed_count(p_rest);
      return;
   }

   /* DR<1-6>:state,batt,age */
   if (('D' == p_id[0]) && ('R' == p_id[1]) &&
       (p_id[DR_DIGIT_OFFSET] >= '1') && (p_id[DR_DIGIT_OFFSET] <= '6'))
   {
      slot = (int)(p_id[DR_DIGIT_OFFSET] - '1');
      parse_reed_sensor(slot, p_rest);
      return;
   }

   /* PIR_COUNT:n — must be checked before the PIR<n> digit test below */
   if (0 == strcmp(p_id, "PIR_COUNT"))
   {
      parse_pir_count(p_rest);
      return;
   }

   /* PIR<1-5>:count,batt,age
    * ID is exactly 4 chars: P I R <digit> */
   if (('P' == p_id[0]) && ('I' == p_id[1]) && ('R' == p_id[2]) &&
       (p_id[PIR_DIGIT_OFFSET] >= '1') && (p_id[PIR_DIGIT_OFFSET] <= '5') &&
       ('\0' == p_id[PIR_DIGIT_OFFSET + 1u]))
   {
      slot = (int)(p_id[PIR_DIGIT_OFFSET] - '1');
      parse_pir_slot(slot, p_rest);
      return;
   }

   /* OCC<1-5>:occupied — per-slot occupancy, mirrors PIR<1-5> pattern.
    * ID is exactly 4 chars: O C C <digit> */
   if (('O' == p_id[0]) && ('C' == p_id[1]) && ('C' == p_id[2]) &&
       (p_id[OCC_DIGIT_OFFSET] >= '1') && (p_id[OCC_DIGIT_OFFSET] <= '5') &&
       ('\0' == p_id[OCC_DIGIT_OFFSET + 1u]))
   {
      slot = (int)(p_id[OCC_DIGIT_OFFSET] - '1');
      (void)parse_int(p_rest, &val);
      ui_set_pir_slot_occupied((uint8_t)slot, (uint8_t)val);
      return;
   }

   /* TEMP_COUNT:n */
   if (0 == strcmp(p_id, "TEMP_COUNT"))
   {
      parse_temp_count(p_rest);
      return;
   }

   /* TEMP<1-4>:decidegc,batt,age
    * ID is exactly 5 chars: T E M P <digit> */
   if (('T' == p_id[0]) && ('E' == p_id[1]) && ('M' == p_id[2]) && ('P' == p_id[3]) &&
       (p_id[TEMP_DIGIT_OFFSET] >= '1') && (p_id[TEMP_DIGIT_OFFSET] <= '4') &&
       ('\0' == p_id[TEMP_DIGIT_OFFSET + 1u]))
   {
      slot = (int)(p_id[TEMP_DIGIT_OFFSET] - '1');
      parse_temp_slot(slot, p_rest);
      return;
   }

   /* DB<0-3>:age_s,online — per-cam doorbell liveness.
    * ID is exactly 3 chars: D B <digit>.
    * Slot is 0-based (- '0') matching hub serialization convention.
    * Must be dispatched before DOORBELL: to avoid prefix collision. */
   if (('D' == p_id[0]) && ('B' == p_id[1]) &&
       (p_id[DB_DIGIT_OFFSET] >= '0') && (p_id[DB_DIGIT_OFFSET] <= '3') &&
       ('\0' == p_id[DB_DIGIT_OFFSET + 1u]))
   {
      slot = (int)(p_id[DB_DIGIT_OFFSET] - '0');
      parse_doorbell_slot(slot, p_rest);
      return;
   }

   /* DOORBELL:pressed,device_id — press event, one-shot */
   if (0 == strcmp(p_id, "DOORBELL"))
   {
      char  tmp[UART_LINE_LEN] = {0};
      char *p_tok              = NULL;
      int   pressed            = 0;
      int   device_id          = 0;

      (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
      tmp[sizeof(tmp) - 1u] = '\0';

      p_tok = strtok(tmp, ",");
      if (NULL != p_tok) { (void)parse_int(p_tok, &pressed); }

      p_tok = strtok(NULL, ",");
      if (NULL != p_tok) { (void)parse_int(p_tok, &device_id); }

      if (0 != pressed)
      {
         char dbg[48];
         snprintf(dbg, sizeof(dbg), "[DOORBELL] pressed dev_id=%d\r\n", device_id);
         log_enqueue(dbg);
         ui_set_doorbell((uint8_t)pressed, (uint8_t)device_id);
      }

      return;
   }

   /* STATE:... */
   if (0 == strcmp(p_id, "STATE"))
   {
      parse_state_message(p_rest);
      return;
   }

   /* Single-value fallthrough — extract optional battery field */
   p_comma = strchr(p_rest, ',');
   if (NULL != p_comma)
   {
      *p_comma = '\0';
      (void)parse_int(p_comma + 1, &batt);
   }

   (void)parse_int(p_rest, &val);
   parse_single_value(p_id, val, batt);
}
