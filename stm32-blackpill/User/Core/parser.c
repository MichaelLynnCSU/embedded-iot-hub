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
 *            REED_COUNT, DR1-DR6, PIR_COUNT, PIR1-PIR5, OCC1-OCC5, PIR,
 *            LGT, LCK, MTR, STATE, OCC, TEMP_COUNT, TEMP1-TEMP4,
 *            DB0-DB9, CAM1-CAM3, DOORBELL.
 *          All parsing uses strtol() rather than atoi() for error detection.
 *
 *          Online/offline pattern (all devices including MTR):
 *          - When online: update value and stamp last-seen tick
 *          - When offline: do nothing — HB_TIMEOUT_MS expiry drives dot red
 *
 * \note    OCC frame (2026-04-30):
 *          OCC:0 / OCC:1 retained for backward compatibility.
 *
 * \note    Per-slot PIR frames (2026-05-XX):
 *          PIR_COUNT:n and PIR<n>:count,batt,age added.
 *
 * \note    Per-slot OCC frames (2026-05-20):
 *          OCC<1-5>:n added. Legacy OCC:n retained.
 *
 * \note    Doorbell liveness (2026-06-09):
 *          DB<0-3>:age_s,online added.
 *
 * \note    Inference camera liveness (2026-06-10):
 *          CAM<1-3>:online added. Slot is 1-based (- '1') matching
 *          uart_controller.c serialization. Calls ui_stamp_cam_online().
 *          ui_set_cam() carries display config only — no liveness.
 *          online==0 is intentionally a no-op: offline is time-driven
 *          by HB_TIMEOUT_MS. See arch spec for rationale.
 *
 * \note    Inference-aware doorbell (2026-06-14):
 *          DOORBELL frame extended to pressed,device_id[,person,conf_pct,asset].
 *          asset is a 16-char ISO 8601 timestamp token (e.g. 20260614T182513Z).
 *          press->inference delta is 1-4 s by design; person/conf/asset arrive
 *          on subsequent pressed=0 frames while g_doorbell_pending is open.
 *          Parser always forwards to ui_set_doorbell_result(); UI layer gates
 *          on g_doorbell_pending.
 *
 * \note    DB<n> slot range widened (2026-06-15):
 *          DB_DIGIT_OFFSET digit check widened from '0'-'3' to '0'-'9' to
 *          match uart_controller.c, which emits DB0..DB<MAX_DOORBELL_CAMS-1>
 *          (0-indexed). parse_doorbell_slot() already bounds-checks the slot
 *          against MAX_DOORBELL_CAMS, matching the PIR/TEMP/CAM slot pattern
 *          (dispatcher does coarse digit routing, handler does real bounds
 *          enforcement). Previously, if MAX_DOORBELL_CAMS > 4, frames like
 *          DB4:... fell through to the single-value handler and were
 *          silently dropped.
 *
 * \note    EID_* frames (2026-06-19):
 *          uart_controller.c now emits one EID_<TYPE><slot>:value line per
 *          active device per bundle (EID_PIR1, EID_REED1, EID_TEMP1,
 *          EID_LOCK, EID_LIGHT) as per-device correlation trace metadata.
 *          The BlackPill LCD renders state — it does not consume event IDs.
 *          EID_* lines are explicitly acknowledged and discarded at the top
 *          of the dispatcher (before parse_int or fallthrough logic) so:
 *            - The protocol contract is documented in code, not just comments.
 *            - parse_int() is not run on a non-numeric EID value string.
 *            - Future developers see an intentional no-op, not a mystery drop.
 *          No EID storage arrays or display widgets are added. If event ID
 *          display is ever needed on the BlackPill, extend here.
 *
 * \note    LGT/LCK age-gated online stamp (2026-06-26):
 *          LGT and LCK moved out of parse_single_value() fallthrough into
 *          dedicated parse_light() and parse_lock() handlers. Both now parse
 *          the age field from the wire frame and gate ui_stamp_dev_online()
 *          on WIRE_AGE_ONLINE_THRESHOLD_S, matching the pattern used by
 *          parse_reed_sensor(), parse_pir_slot(), and parse_temp_slot().
 *          LGT wire format: val,age  (no batt — wired to wall outlet).
 *          LCK wire format: val,batt,age  (BLE, has battery).
 *          Previously, both stamped online unconditionally on every received
 *          frame, so a powered-off device with a stale hub-side age would
 *          appear permanently online on the LCD.
 *
 * \note    Wire age setters (2026-06-26):
 *          parse_light() now calls ui_set_light_age(age).
 *          parse_lock()  now calls ui_set_lock_age(age).
 *          parse_state_message() now calls ui_set_pir_age(f[4]),
 *            ui_set_light_age(f[5]), ui_set_lock_age(f[6]) when present.
 *
 *          TWO SEPARATE CONCERNS — do not conflate:
 *
 *          1. ui_set_<dev>_age(age)  — stores the age VALUE from the wire
 *             frame into g_home.<dev>_age. Used by ui_system.c to display
 *             "A:Xs". Has no effect on online/offline status.
 *
 *          2. ui_stamp_dev_online(dev, now)  — writes a local HAL_GetTick()
 *             timestamp into g_dev_last_seen[dev]. Used by ui_update() to
 *             compute g_dev_online[] via HB_TIMEOUT_MS. Drives the green/red
 *             dot. Has no effect on the displayed age value.
 *
 *          Both are called in the same frame handler but they are independent.
 *          Calling one does NOT imply or replace the other.
 *
 *          MTR (motor) has no age field in its wire frame today. motor_age
 *          stays AGE_UNKNOWN_VAL until the MTR frame gains an age column.
 *          temp_age is set from f[4] in STATE only (no dedicated TEMP frame
 *          currently carries a scalar aggregate age).
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

/* NOTE: WIRE_AGE_ONLINE_THRESHOLD_S is intentionally distinct from
 * BLE_AGE_THRESHOLD_S in blackpill_logic.h (150 s). That constant is used
 * by the UI layer for liveness display logic. This one (300 s) gates whether
 * the parser stamps a device online from the age field carried in incoming
 * wire frames. Do NOT merge them — they serve different purposes at different
 * layers. If you change one, verify the other is still correct for its use
 * case.                                                                      */
#define WIRE_AGE_ONLINE_THRESHOLD_S  300u

#define STATE_FIELD_COUNT      8u
#define REED_SLOT_MIN          0
#define DR_DIGIT_OFFSET        2u
#define PIR_DIGIT_OFFSET       3u
#define OCC_DIGIT_OFFSET       3u
#define TEMP_DIGIT_OFFSET      4u
#define DB_DIGIT_OFFSET        2u
#define CAM_DIGIT_OFFSET       3u  /**< Character offset of digit in "CAM1" */

/************************** STATIC (PRIVATE) FUNCTIONS ************************/

static int parse_int(const char *p_str, int *p_result)
{
   char   *p_end = NULL;
   long    val   = 0;

   if ((NULL == p_str) || (NULL == p_result)) { return -1; }

   errno = 0;
   val   = strtol(p_str, &p_end, 10);

   if ((errno != 0) || (p_end == p_str)) { return -1; }

   *p_result = (int)val;
   return 0;
}

static void parse_reed_count(const char *p_rest)
{
   int  n           = 0;
   char log_buf[48] = {0};

   if (0 != parse_int(p_rest, &n)) { return; }
   if (n > (int)MAX_REEDS)         { n = (int)MAX_REEDS; }

   if ((uint8_t)n != ui_get_reed_count())
   {
      ui_set_reed_count((uint8_t)n);
      ui_reflow(n);
      (void)snprintf(log_buf, sizeof(log_buf), "[UI] Reflow reed_count=%d\r\n", n);
      log_enqueue(log_buf);
   }
}

static void parse_pir_count(const char *p_rest)
{
   int  n           = 0;
   char log_buf[48] = {0};

   if (0 != parse_int(p_rest, &n)) { return; }
   if (n > (int)MAX_PIRS)          { n = (int)MAX_PIRS; }
   if (n < 0)                      { n = 0; }

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
   int  n           = 0;
   char log_buf[48] = {0};

   if (0 != parse_int(p_rest, &n)) { return; }
   if (n > (int)MAX_TEMPS)         { n = (int)MAX_TEMPS; }
   if (n < 0)                      { n = 0; }

   if ((uint8_t)n != ui_get_temp_count_slots())
   {
      ui_set_temp_count_slots((uint8_t)n);
      ui_reflow_temp(n);
      (void)snprintf(log_buf, sizeof(log_buf), "[UI] Reflow temp_count=%d\r\n", n);
      log_enqueue(log_buf);
   }
}

static void parse_reed_sensor(int slot, const char *p_rest)
{
   char     tmp[UART_LINE_LEN] = {0};
   char    *p_tok              = NULL;
   int      state              = -1;
   int      batt               = -1;
   int      age                = 0xFFFF;
   uint32_t now                = 0u;

   if ((slot < REED_SLOT_MIN) || (slot >= (int)MAX_REEDS)) { return; }
   if (NULL == p_rest) { return; }

   (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
   tmp[sizeof(tmp) - 1u] = '\0';

   p_tok = strtok(tmp, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &state); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &batt); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &age); }

   now = HAL_GetTick();

   if (state >= 0) { ui_set_reed_state((uint8_t)slot, (uint8_t)state); }
   ui_set_reed_batt((uint8_t)slot, (int8_t)batt);
   ui_set_reed_age((uint8_t)slot,  (uint16_t)age);

   if (age < (int)WIRE_AGE_ONLINE_THRESHOLD_S)
   {
      ui_stamp_reed_online((uint8_t)slot, now);
   }

   (void)snprintf(tmp, sizeof(tmp),
                  "[REED] slot=%d st=%d batt=%d age=%d\r\n",
                  slot, state, batt, age);
   log_enqueue(tmp);
}

static void parse_pir_slot(int slot, const char *p_rest)
{
   char     tmp[UART_LINE_LEN] = {0};
   char    *p_tok              = NULL;
   int      count              = 0;
   int      batt               = -1;
   int      age                = 0xFFFF;
   uint32_t now                = 0u;

   if ((slot < 0) || (slot >= (int)MAX_PIRS)) { return; }
   if (NULL == p_rest) { return; }

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

   if (age < (int)WIRE_AGE_ONLINE_THRESHOLD_S)
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

   if ((slot < 0) || (slot >= (int)MAX_TEMPS)) { return; }
   if (NULL == p_rest) { return; }

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

   if (age < (int)WIRE_AGE_ONLINE_THRESHOLD_S)
   {
      ui_stamp_temp_online((uint8_t)slot, now);
   }

   char dbg[96];
   snprintf(dbg, sizeof(dbg),
            "[TEMP] slot=%d decidegc=%d batt=%d age=%d\r\n",
            slot, temp_decidegc, batt, age);
   log_enqueue(dbg);
}

static void parse_doorbell_slot(int slot, const char *p_rest)
{
   char     tmp[UART_LINE_LEN] = {0};
   char    *p_tok              = NULL;
   int      age_s              = 0xFFFF;
   uint32_t now                = 0u;

   if ((slot < 0) || (slot >= (int)MAX_DOORBELL_CAMS)) { return; }
   if (NULL == p_rest) { return; }

   (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
   tmp[sizeof(tmp) - 1u] = '\0';

   p_tok = strtok(tmp, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &age_s); }

   now = HAL_GetTick();

   ui_set_doorbell_slot_age((uint8_t)slot, (uint16_t)age_s);

   if (age_s < (int)WIRE_AGE_ONLINE_THRESHOLD_S)
   {
      ui_stamp_doorbell_online((uint8_t)slot, now);
   }

   char dbg[64];
   snprintf(dbg, sizeof(dbg),
            "[DB] slot=%d age_s=%d\r\n",
            slot, age_s);
   log_enqueue(dbg);
}

static void parse_cam_slot(int slot, const char *p_rest)
{
   char     tmp[UART_LINE_LEN] = {0};
   char    *p_tok              = NULL;
   int      age                = 0xFFFF;
   uint32_t now                = 0u;

   if ((slot < 0) || (slot >= (int)MAX_CAMS)) { return; }
   if (NULL == p_rest) { return; }

   (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
   tmp[sizeof(tmp) - 1u] = '\0';

   p_tok = strtok(tmp, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &age); }

   now = HAL_GetTick();

   ui_set_cam((uint8_t)slot);

   /* age_s = seconds since camera last sent UDP heartbeat to camera_manager.
    * Computed on BBB side: now - last_seen. Counts 0->N, resets on heartbeat.
    * Used for "A:Xs" display and CAM row visibility in ui_system.c.  */




   ui_set_cam_age((uint8_t)slot, (uint16_t)age);

   if (age < (int)WIRE_AGE_ONLINE_THRESHOLD_S)
   {
      ui_stamp_cam_online((uint8_t)slot, now);
   }

   char dbg[64];
   snprintf(dbg, sizeof(dbg),
            "[CAM] slot=%d age=%d\r\n",
            slot, age);
   log_enqueue(dbg);
}

static void parse_light(const char *p_rest)
{
   char     tmp[UART_LINE_LEN] = {0};
   char    *p_tok              = NULL;
   int      val                = 0;
   int      age                = 0xFFFF;
   uint32_t now                = 0u;

   if (NULL == p_rest) { return; }

   (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
   tmp[sizeof(tmp) - 1u] = '\0';

   /* LGT wire format: val,age  (no batt — wired to wall outlet) */
   p_tok = strtok(tmp, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &val); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &age); }

   now = HAL_GetTick();

   ui_set_light((uint8_t)val);

   /* Store wire age for SYSTEM view display ("A:Xs").
    * This is the age the ESP32 hub reports for its BLE/WiFi light device —
    * NOT a locally computed elapsed value.
    * It does NOT affect the online/offline dot; that is driven by the
    * ui_stamp_dev_online() call below + HB_TIMEOUT_MS in ui_update().   */
   ui_set_light_age((uint16_t)age);

   if (age < (int)WIRE_AGE_ONLINE_THRESHOLD_S)
   {
      ui_stamp_dev_online(eDEV_LIGHT, now);
   }

   char dbg[48];
   snprintf(dbg, sizeof(dbg), "[LGT] val=%d age=%d\r\n", val, age);
   log_enqueue(dbg);
}

static void parse_lock(const char *p_rest)
{
   char     tmp[UART_LINE_LEN] = {0};
   char    *p_tok              = NULL;
   int      val                = 0;
   int      batt               = -1;
   int      age                = 0xFFFF;
   uint32_t now                = 0u;

   if (NULL == p_rest) { return; }

   (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
   tmp[sizeof(tmp) - 1u] = '\0';

   /* LCK wire format: val,batt,age  (BLE, has battery) */
   p_tok = strtok(tmp, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &val); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &batt); }

   p_tok = strtok(NULL, ",");
   if (NULL != p_tok) { (void)parse_int(p_tok, &age); }

   now = HAL_GetTick();

   ui_set_lock((uint8_t)val);
   if (batt >= 0) { ui_set_lock_batt((int8_t)batt); }

   /* Store wire age for SYSTEM view display ("A:Xs").
    * This is the age the ESP32 hub reports for its BLE lock device —
    * NOT a locally computed elapsed value.
    * It does NOT affect the online/offline dot; that is driven by the
    * ui_stamp_dev_online() call below + HB_TIMEOUT_MS in ui_update().   */
   ui_set_lock_age((uint16_t)age);

   if (age < (int)WIRE_AGE_ONLINE_THRESHOLD_S)
   {
      ui_stamp_dev_online(eDEV_LOCK, now);
   }

   char dbg[56];
   snprintf(dbg, sizeof(dbg), "[LCK] val=%d batt=%d age=%d\r\n", val, batt, age);
   log_enqueue(dbg);
}

static void parse_state_message(const char *p_rest)
{
   char     tb[UART_LINE_LEN]   = {0};
   char    *p_tok               = NULL;
   int      f[STATE_FIELD_COUNT];
   uint8_t  i                   = 0u;
   uint32_t now                 = 0u;
   int      old_proto           = 0;
   int      n                   = 0;

   if (NULL == p_rest) { return; }

   for (i = 0u; i < STATE_FIELD_COUNT; i++) { f[i] = -1; }

   (void)strncpy(tb, p_rest, sizeof(tb) - 1u);
   tb[sizeof(tb) - 1u] = '\0';

   p_tok = strtok(tb, ",");
   for (i = 0u; (i < STATE_FIELD_COUNT) && (NULL != p_tok); i++)
   {
      (void)parse_int(p_tok, &f[i]);
      p_tok = strtok(NULL, ",");
   }

   /* STATE field map:
    *   f[0] = temp        f[1] = pir_count  f[2] = light_val  f[3] = lock_val
    *   f[4] = age_pir     f[5] = age_light   f[6] = age_lock   f[7] = reed_count */

   now = HAL_GetTick();

   if (f[0] >= 0) { ui_set_temp((uint8_t)f[0]);       }
   if (f[1] >= 0) { ui_set_pir_count((uint32_t)f[1]); }
   if (f[2] >= 0) { ui_set_light((uint8_t)f[2]);      }
   if (f[3] >= 0) { ui_set_lock((uint8_t)f[3]);       }

   /* Store wire ages for SYSTEM view display.
    * f[4]=age_pir, f[5]=age_light, f[6]=age_lock — present in new protocol
    * only (old_proto=1 when f[4]<0). These are the ages the ESP32 hub
    * reports for each BLE/WiFi device, stored into g_home.*_age fields and
    * displayed as "A:Xs" in ui_system.c.
    *
    * They are independent of the ui_stamp_dev_online() calls below.
    * Storing the wire age does NOT stamp a device online; stamping online
    * does NOT update the displayed age. Both happen in the same frame
    * but for entirely different consumers. See parser.c file-level note. */
   if (f[4] >= 0) { ui_set_pir_age((uint16_t)f[4]);   }
   if (f[5] >= 0) { ui_set_light_age((uint16_t)f[5]); }
   if (f[6] >= 0) { ui_set_lock_age((uint16_t)f[6]);  }

   ui_stamp_dev_online(eDEV_TEMP, now);

   old_proto = (f[4] < 0);

   if ((0 != old_proto) || (f[4] < (int)WIRE_AGE_ONLINE_THRESHOLD_S))
   {
      ui_stamp_dev_online(eDEV_PIR, now);
   }

   if ((0 != old_proto) || (f[5] < (int)WIRE_AGE_ONLINE_THRESHOLD_S))
   {
      ui_stamp_dev_online(eDEV_LIGHT, now);
   }

   if ((0 != old_proto) || (f[6] < (int)WIRE_AGE_ONLINE_THRESHOLD_S))
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

static void parse_single_value(const char *p_id, int val, int batt)
{
   uint32_t now = HAL_GetTick();

   if (NULL == p_id) { return; }

   if (0 == strcmp(p_id, "PIR"))
   {
      /* Legacy aggregate PIR frame (PIR:count,batt) — no age field.
       * pir_age stays AGE_UNKNOWN_VAL until STATE frame f[4] or a
       * per-slot PIR<n> frame provides it.                            */
      ui_set_pir_count((uint32_t)val);
      if (batt >= 0) { ui_set_pir_batt((uint8_t)batt); }
   }
   else if (0 == strcmp(p_id, "MTR"))
   {
      /* MTR frame (MTR:val,batt) — no age field in current protocol.
       * motor_age stays AGE_UNKNOWN_VAL until the MTR frame gains an
       * age column. ui_set_motor_age() exists and will be called here
       * once the protocol is extended.                                */
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
      ui_set_pir_occupied((uint8_t)val);
   }
}

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  Parse one null-terminated ASCII telemetry line from the ESP32.
 *
 * \param  p_line - Null-terminated message string, e.g. "PIR:42,87".
 *
 * \details Dispatch order:
 *          1.  EID_*       — trace metadata, explicitly ignored, returns.
 *          2.  REED_COUNT  — reed count handler, returns.
 *          3.  DR<1-6>     — reed slot handler, returns.
 *          4.  PIR_COUNT   — PIR count handler, returns.
 *          5.  PIR<1-5>    — PIR slot handler, returns.
 *          6.  OCC<1-5>    — occupancy slot handler, returns.
 *          7.  TEMP_COUNT  — temp count handler, returns.
 *          8.  TEMP<1-4>   — temp slot handler, returns.
 *          9.  DB<0-9>     — doorbell liveness handler, returns.
 *          10. CAM<1-3>    — inference camera liveness handler, returns.
 *          11. DOORBELL    — press/inference event handler, returns.
 *          12. LGT         — light state/age/liveness handler, returns.
 *          13. LCK         — lock state/age/liveness handler, returns.
 *          14. STATE       — bulk state/age handler, returns.
 *          15. Everything else — single-value fallthrough (PIR, MTR, OCC).
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

   char raw[UART_LINE_LEN + 16];
   snprintf(raw, sizeof(raw), "[RX] %.40s\r\n", p_line);
   log_enqueue(raw);

   if (NULL == p_line) { return; }

   (void)strncpy(buf, p_line, sizeof(buf) - 1u);
   buf[sizeof(buf) - 1u] = '\0';

   p_colon = strchr(buf, ':');
   if (NULL == p_colon) { return; }

   *p_colon = '\0';
   p_id     = buf;
   p_rest   = p_colon + 1;

   /* EID_* — per-device event trace metadata emitted by uart_controller.c.
    * The BlackPill LCD renders state only — it does not consume event IDs.
    * Acknowledged explicitly here so the protocol contract is visible in
    * code and parse_int() / fallthrough logic are never run on EID values.
    * See uart_controller.c note "Per-device event_id tracing (2026-06-19)".
    * Extend here if event ID display is ever added to the BlackPill UI.   */
   if (0 == strncmp(p_id, "EID_", 4u))
   {
      return;
   }

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

   /* PIR_COUNT:n — before PIR<n> digit test */
   if (0 == strcmp(p_id, "PIR_COUNT"))
   {
      parse_pir_count(p_rest);
      return;
   }

   /* PIR<1-5>:count,batt,age */
   if (('P' == p_id[0]) && ('I' == p_id[1]) && ('R' == p_id[2]) &&
       (p_id[PIR_DIGIT_OFFSET] >= '1') && (p_id[PIR_DIGIT_OFFSET] <= '5') &&
       ('\0' == p_id[PIR_DIGIT_OFFSET + 1u]))
   {
      slot = (int)(p_id[PIR_DIGIT_OFFSET] - '1');
      parse_pir_slot(slot, p_rest);
      return;
   }

   /* OCC<1-5>:occupied */
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

   /* TEMP<1-4>:decidegc,batt,age */
   if (('T' == p_id[0]) && ('E' == p_id[1]) && ('M' == p_id[2]) && ('P' == p_id[3]) &&
       (p_id[TEMP_DIGIT_OFFSET] >= '1') && (p_id[TEMP_DIGIT_OFFSET] <= '4') &&
       ('\0' == p_id[TEMP_DIGIT_OFFSET + 1u]))
   {
      slot = (int)(p_id[TEMP_DIGIT_OFFSET] - '1');
      parse_temp_slot(slot, p_rest);
      return;
   }

   /* DB<0-9>:age_s,online — must be before DOORBELL to avoid prefix collision.
    * Digit range intentionally wide (0-9); parse_doorbell_slot() bounds-checks
    * against MAX_DOORBELL_CAMS, matching the PIR/TEMP/CAM slot pattern. */
   if (('D' == p_id[0]) && ('B' == p_id[1]) &&
       (p_id[DB_DIGIT_OFFSET] >= '0') && (p_id[DB_DIGIT_OFFSET] <= '9') &&
       ('\0' == p_id[DB_DIGIT_OFFSET + 1u]))
   {
      slot = (int)(p_id[DB_DIGIT_OFFSET] - '0');
      parse_doorbell_slot(slot, p_rest);
      return;
   }

   /* CAM<1-3>:online — 1-based slot matching uart_controller.c */
   if (('C' == p_id[0]) && ('A' == p_id[1]) && ('M' == p_id[2]) &&
       (p_id[CAM_DIGIT_OFFSET] >= '1') && (p_id[CAM_DIGIT_OFFSET] <= '3') &&
       ('\0' == p_id[CAM_DIGIT_OFFSET + 1u]))
   {
      slot = (int)(p_id[CAM_DIGIT_OFFSET] - '1');
      parse_cam_slot(slot, p_rest);
      return;
   }

   /* DOORBELL:pressed,device_id[,person,conf_pct,asset]
    *
    * asset is a 16-char ISO 8601 timestamp token (e.g. 20260614T182513Z).
    * press->inference delta is 1-4 s by design; person/conf/asset will be
    * absent or zero on the first (pressed=1) frame and arrive on a later
    * pressed=0 frame while g_doorbell_pending is open in ui.c.
    * Parser always forwards regardless of pressed value — the UI layer gates
    * on g_doorbell_pending. Parser stays a pure parser.                     */
   if (0 == strcmp(p_id, "DOORBELL"))
   {
      char  tmp[UART_LINE_LEN] = {0};
      char *p_tok              = NULL;
      int   pressed            = 0;
      int   device_id          = 0;
      int   person             = 0;
      int   conf_pct           = 0;
      char  asset[20]          = {0};  /* 16-char token + null; 3 bytes margin */

      (void)strncpy(tmp, p_rest, sizeof(tmp) - 1u);
      tmp[sizeof(tmp) - 1u] = '\0';

      p_tok = strtok(tmp, ",");
      if (NULL != p_tok) { (void)parse_int(p_tok, &pressed);   }

      p_tok = strtok(NULL, ",");
      if (NULL != p_tok) { (void)parse_int(p_tok, &device_id); }

      p_tok = strtok(NULL, ",");
      if (NULL != p_tok) { (void)parse_int(p_tok, &person);    }

      p_tok = strtok(NULL, ",");
      if (NULL != p_tok) { (void)parse_int(p_tok, &conf_pct);  }

      p_tok = strtok(NULL, ",");
      if (NULL != p_tok)
      {
         (void)strncpy(asset, p_tok, sizeof(asset) - 1u);
         asset[sizeof(asset) - 1u] = '\0';
      }

      ui_set_doorbell_result((uint8_t)pressed, (uint8_t)device_id,
                             (uint8_t)person,  (uint8_t)conf_pct,
                             asset);

      char dbg[80];

      if (0 != pressed)
      {
         snprintf(dbg, sizeof(dbg),
                  "[DOORBELL] press dev=%d\r\n", device_id);
         log_enqueue(dbg);
      }

      if ('\0' != asset[0])
      {
         snprintf(dbg, sizeof(dbg),
                  "[DOORBELL] result dev=%d person=%d conf=%d asset=%.19s\r\n",
                  device_id, person, conf_pct, asset);
         log_enqueue(dbg);
      }

      return;
   }

   /* LGT:val,age — wired to wall outlet, no batt field.
    * Stores wire age via ui_set_light_age() for SYSTEM view display.
    * Stamps eDEV_LIGHT online only when age < WIRE_AGE_ONLINE_THRESHOLD_S. */
   if (0 == strcmp(p_id, "LGT"))
   {
      parse_light(p_rest);
      return;
   }

   /* LCK:val,batt,age — BLE smart lock, has battery.
    * Stores wire age via ui_set_lock_age() for SYSTEM view display.
    * Stamps eDEV_LOCK online only when age < WIRE_AGE_ONLINE_THRESHOLD_S. */
   if (0 == strcmp(p_id, "LCK"))
   {
      parse_lock(p_rest);
      return;
   }

   /* STATE:tmp,pir,lgt,lck,age_pir,age_lgt,age_lck,reed_count
    * Stores wire ages f[4]/f[5]/f[6] via ui_set_{pir,light,lock}_age().
    * Online stamps are gated on WIRE_AGE_ONLINE_THRESHOLD_S independently. */
   if (0 == strcmp(p_id, "STATE"))
   {
      parse_state_message(p_rest);
      return;
   }

   /* Single-value fallthrough (PIR, MTR, OCC) */
   p_comma = strchr(p_rest, ',');
   if (NULL != p_comma)
   {
      *p_comma = '\0';
      (void)parse_int(p_comma + 1, &batt);
   }

   (void)parse_int(p_rest, &val);
   parse_single_value(p_id, val, batt);
}
