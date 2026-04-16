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
 *            REED_COUNT, DR1-DR6, PIR, LGT, LCK, MTR, STATE.
 *          All parsing uses strtol() rather than atoi() for error detection.
 *
 *          Online/offline pattern (all devices including MTR):
 *          - When online: update value and stamp last-seen tick
 *          - When offline: do nothing — HB_TIMEOUT_MS expiry drives dot red
 *          - This is identical behaviour for PIR, LGT, LCK, and MTR
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

#define BLE_AGE_THRESHOLD_S  150u  /**< Seconds before a BLE device is stale */
#define STATE_FIELD_COUNT      8u  /**< Number of fields in a STATE message   */
#define REED_SLOT_MIN          0   /**< First valid reed slot index            */
#define DR_ID_PREFIX_LEN       2u  /**< Length of "DR" prefix in DRn IDs      */
#define DR_DIGIT_OFFSET        2u  /**< Character offset of digit in "DR1"     */

/************************** STATIC (PRIVATE) FUNCTIONS ************************/

/**
 * \brief  Convert an ASCII string to int using strtol with overflow check.
 *
 * \param  p_str    - Null-terminated string to convert.
 * \param  p_result - Output: parsed integer value on success.
 *
 * \return 0 on success, -1 on parse error or overflow.
 *
 * \author MichaelLynnCSU
 */
static int parse_int(const char *p_str, int *p_result)
{
   char   *p_end   = NULL; /**< Points past last converted character */
   long    val     = 0;    /**< strtol result                        */

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
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void parse_reed_count(const char *p_rest)
{
   int     n          = 0;     /**< Parsed reed count                    */
   char    log_buf[48] = {0};  /**< Formatted log string                 */

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
 * \brief  Handle a DRn:state,batt,age message — update one reed sensor slot.
 *
 * \param  slot   - Zero-based reed index (0..MAX_REEDS-1).
 * \param  p_rest - String after the colon: "state,batt,age".
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void parse_reed_sensor(int slot, const char *p_rest)
{
   char    tmp[UART_LINE_LEN] = {0}; /**< Mutable copy for strtok          */
   char   *p_tok              = NULL; /**< strtok pointer                   */
   int     state              = -1;  /**< Parsed door state (0=closed,1=open)*/
   int     batt               = -1;  /**< Parsed battery percentage         */
   int     age                = 0xFFFF; /**< Parsed BLE age in seconds      */
   uint32_t now               = 0u;  /**< Current tick in ms                */

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
   if (NULL != p_tok)
   {
      (void)parse_int(p_tok, &state);
   }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok)
   {
      (void)parse_int(p_tok, &batt);
   }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok)
   {
      (void)parse_int(p_tok, &age);
   }

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
 * \brief  Handle a STATE:tmp,pir,lgt,lck,age_pir,age_lgt,age_lck,reed_count
 *         message — bulk sensor update from ESP32.
 *
 * \param  p_rest - String after the colon containing 8 comma-delimited fields.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void parse_state_message(const char *p_rest)
{
   char    tb[UART_LINE_LEN]         = {0}; /**< Mutable copy for strtok    */
   char   *p_tok                     = NULL; /**< strtok pointer             */
   int     f[STATE_FIELD_COUNT];             /**< Parsed field array         */
   uint8_t i                         = 0u;  /**< Field index                */
   uint32_t now                      = 0u;  /**< Current tick ms            */
   int     old_proto                 = 0;   /**< Non-zero if legacy 3-field  */
   int     n                         = 0;   /**< Reed count from STATE field */

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

   if (f[0] >= 0) { ui_set_temp((uint8_t)f[0]);         }
   if (f[1] >= 0) { ui_set_pir_count((uint32_t)f[1]);   }
   if (f[2] >= 0) { ui_set_light((uint8_t)f[2]);        }
   if (f[3] >= 0) { ui_set_lock((uint8_t)f[3]);         }

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
      if (n > (int)MAX_REEDS)
      {
         n = (int)MAX_REEDS;
      }

      if ((uint8_t)n != ui_get_reed_count())
      {
         ui_set_reed_count((uint8_t)n);
         ui_reflow(n);
      }
   }
}

/**
 * \brief  Handle a single-value message: PIR, LGT, LCK, or MTR.
 *
 * \param  p_id   - Null-terminated message identifier string.
 * \param  val    - Primary integer value already parsed from the message.
 * \param  batt   - Battery percentage, or -1 if not present.
 *
 * \return void
 *
 * \details All devices follow the same online/offline pattern:
 *          - Online: update value and stamp last-seen tick
 *          - Offline: do nothing — HB_TIMEOUT_MS expiry turns dot red
 *
 *          MTR follows this exactly. val=1 updates the motor state and
 *          stamps online. val=0 does nothing at all — the last known
 *          motor state stays on screen and the dot fades to red after
 *          HB_TIMEOUT_MS, identical to PIR, LGT, and LCK behaviour.
 *
 * \author MichaelLynnCSU
 */
static void parse_single_value(const char *p_id, int val, int batt)
{
   uint32_t now = HAL_GetTick(); /**< Current tick ms */

   if (NULL == p_id)
   {
      return;
   }

   if (0 == strcmp(p_id, "PIR"))
   {
      ui_set_pir_count((uint32_t)val);
      if (batt >= 0)
      {
         ui_set_pir_batt((uint8_t)batt);
      }
   }
   else if (0 == strcmp(p_id, "LGT"))
   {
      ui_set_light((uint8_t)val);
   }
   else if (0 == strcmp(p_id, "LCK"))
   {
      ui_set_lock((uint8_t)val);
      if (batt >= 0)
      {
         ui_set_lock_batt((int8_t)batt);
      }
   }
   else if (0 == strcmp(p_id, "MTR"))
   {
      if (1 == val)
      {
         ui_set_motor(1u);
         ui_stamp_dev_online(eDEV_MOTOR, now);
      }
      if (batt > 0)
      {
        ui_set_motor_batt(batt);
      }
      /* val=0: do nothing — last known motor state stays on screen.
       * HB_TIMEOUT_MS expiry drives the dot red, identical to how
       * PIR, LGT, and LCK go offline. No explicit offline action needed. */
   }
   else
   {
      /* Unrecognised single-value ID — discard silently */
   }
}

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  Parse one null-terminated ASCII telemetry line from the ESP32.
 *
 * \param  p_line - Null-terminated message string, e.g. "PIR:42,87".
 *
 * \return void
 *
 * \details Splits on the first ':' to extract the message ID, then delegates
 *          to the appropriate static sub-parser. Unknown IDs are silently
 *          discarded.
 *
 * \author MichaelLynnCSU
 */
void parser_process_line(const char *p_line)
{
   char    buf[UART_LINE_LEN] = {0}; /**< Mutable working copy of the line  */
   char   *p_colon            = NULL; /**< Points to ':' separator           */
   char   *p_id               = NULL; /**< Points to message ID string       */
   char   *p_rest             = NULL; /**< Points to value string after ':'  */
   char   *p_comma            = NULL; /**< Points to optional ',' separator  */
   int     val                = 0;   /**< Primary parsed integer value       */
   int     batt               = -1;  /**< Optional battery field value       */
   int     slot               = 0;   /**< Reed slot index for DRn messages   */

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

   if (0 == strcmp(p_id, "REED_COUNT"))
   {
      parse_reed_count(p_rest);
      return;
   }

   if (('D' == p_id[0]) && ('R' == p_id[1]) &&
       (p_id[DR_DIGIT_OFFSET] >= '1') && (p_id[DR_DIGIT_OFFSET] <= '6'))
   {
      slot = (int)(p_id[DR_DIGIT_OFFSET] - '1');
      parse_reed_sensor(slot, p_rest);
      return;
   }

   if (0 == strcmp(p_id, "STATE"))
   {
      parse_state_message(p_rest);
      return;
   }

   /* Single-value messages: extract optional battery field */
   p_comma = strchr(p_rest, ',');
   if (NULL != p_comma)
   {
      *p_comma = '\0';
      (void)parse_int(p_comma + 1, &batt);
   }

   (void)parse_int(p_rest, &val);
   parse_single_value(p_id, val, batt);
}
