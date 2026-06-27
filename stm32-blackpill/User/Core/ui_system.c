/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ui_system.c
 * \author  MichaelLynnCSU
 * \date    2026-06-14
 *
 * \brief   SYSTEM view — scrollable device list creation and refresh.
 *
 * \details Row layout (fixed order, zero-indexed):
 *           0        TEMP  (aggregate)
 *           1        MOTOR
 *           2        LIGHT
 *           3        LOCK
 *           4..7     TEMP1..TEMP4  (hidden until temp_count arrives)
 *           8..12    PIR1..PIR5    (hidden until pir_count arrives)
 *           13..18   DOOR1..DOOR6
 *           19..22   DB0..DB3      (hidden until first DB frame)
 *           23..25   CAM1..CAM3
 *
 * \note    Wire age vs last-seen elapsed (2026-06-26):
 *          Rows 0-3 (TEMP, MOTOR, LIGHT, LOCK) previously computed "A:Xs"
 *          by taking (now - g_dev_last_seen[]) / 1000. This measured how
 *          long ago the STM32 received a frame — i.e. local elapsed time —
 *          NOT the age the ESP32 hub reported for its BLE/WiFi device.
 *
 *          Fixed: rows 0-3 now read g_home.{temp,motor,light,lock}_age,
 *          which are the wire age values stored by ui_set_<dev>_age() in
 *          parser.c. This matches the pattern already used by PIR slots,
 *          reed/door slots, and doorbell slots.
 *
 *          The green/red dot colour for these rows is still driven by
 *          g_dev_online[], which is computed in ui_update() from
 *          g_dev_last_seen[] + HB_TIMEOUT_MS. That is correct and unchanged.
 *          Wire age and online/offline dot are independent.
 *
 *          CAM rows are unaffected: they have no wire age field (the CAM
 *          frame carries only online=0/1) so they continue using local
 *          elapsed from g_cam_last_seen[]. That elapsed value is accurate
 *          for cameras because it is the only age information available.
 *
 * \note    TEMP slot age display (2026-06-27):
 *          TEMP slot rows now show "A:Xs" from g_home.temp_slot_age[],
 *          matching the pattern used by PIR and reed slot rows.
 *          Previously the slot rows showed only temp and batt with no age.
 *          g_home.temp_age (aggregate row 0) is now promoted from slot 0
 *          inside ui_set_temp_slot_age() in ui.c — see that file's note
 *          "Slot-to-scalar promotion (2026-06-27)".
 ******************************************************************************/

#include "ui_priv.h"

/************************ LIST OBJECTS (externed in ui_priv.h) ****************/

lv_obj_t *g_sys_list                = NULL;
lv_obj_t *g_sys_rows[SYS_ROW_COUNT];

/************************ STATIC (PRIVATE) FUNCTIONS **************************/

static void sys_row_create(lv_obj_t *p_parent, lv_obj_t **pp_row,
                            const char *p_text, uint8_t hidden)
{
   *pp_row = lv_label_create(p_parent);
   lv_label_set_text(*pp_row, p_text);
   lv_obj_set_style_text_color(*pp_row, lv_color_white(), 0);
   lv_obj_set_width(*pp_row, (lv_coord_t)FULL_TILE_WIDTH);
   lv_label_set_long_mode(*pp_row, LV_LABEL_LONG_CLIP);
   lv_obj_clear_flag(*pp_row, LV_OBJ_FLAG_SCROLLABLE);
   if (0u != hidden) { lv_obj_add_flag(*pp_row, LV_OBJ_FLAG_HIDDEN); }
}

static void sys_row_set(uint8_t row, const char *p_buf, uint8_t online)
{
   if (NULL == g_sys_rows[row]) { return; }
   lv_label_set_text(g_sys_rows[row], p_buf);
   lv_obj_set_style_text_color(g_sys_rows[row],
      (0u != online) ? lv_color_hex(C_ROW_ONLINE) : lv_color_hex(C_ROW_OFFLINE),
      0);
}

/************************** PUBLIC FUNCTIONS ***********************************/

void create_sys_list(lv_obj_t *p_scr)
{
   char    name[16];
   uint8_t i   = 0u;
   uint8_t row = 0u;

   if (NULL == p_scr) { return; }

   g_sys_list = lv_obj_create(p_scr);
   lv_obj_set_pos(g_sys_list, (int)TILE_LEFT_MARGIN, (int)CONTENT_TOP);
   lv_obj_set_size(g_sys_list, (int)FULL_TILE_WIDTH, (int)CONTENT_H);
   lv_obj_set_style_bg_opa(g_sys_list, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(g_sys_list, 0, 0);
   lv_obj_set_style_pad_all(g_sys_list, 0, 0);
   lv_obj_set_style_pad_row(g_sys_list, 6, 0);

   lv_obj_add_flag(g_sys_list,   LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_clear_flag(g_sys_list, LV_OBJ_FLAG_SCROLL_CHAIN);
   lv_obj_set_scroll_dir(g_sys_list, LV_DIR_VER);
   lv_obj_set_scrollbar_mode(g_sys_list, LV_SCROLLBAR_MODE_ACTIVE);
   lv_obj_set_scroll_snap_y(g_sys_list, LV_SCROLL_SNAP_NONE);
   lv_obj_set_flex_flow(g_sys_list, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_flex_align(g_sys_list, LV_FLEX_ALIGN_START,
                                      LV_FLEX_ALIGN_START,
                                      LV_FLEX_ALIGN_START);

   /* Fixed rows */
   const char *const fixed[4] = { "TEMP", "MOTOR", "LIGHT", "LOCK" };
   for (i = 0u; i < 4u; i++)
   {
      sys_row_create(g_sys_list, &g_sys_rows[row], fixed[i], 0u);
      row++;
   }

   /* BLE temp slots — hidden until TEMP_COUNT arrives */
   for (i = 0u; i < (uint8_t)MAX_TEMPS; i++)
   {
      (void)snprintf(name, sizeof(name), "TEMP %u", (unsigned int)(i + 1u));
      sys_row_create(g_sys_list, &g_sys_rows[row], name, 1u);
      row++;
   }

   /* PIR slots */
   for (i = 0u; i < (uint8_t)MAX_PIRS; i++)
   {
      (void)snprintf(name, sizeof(name), "PIR %u", (unsigned int)(i + 1u));
      sys_row_create(g_sys_list, &g_sys_rows[row], name, 0u);
      row++;
   }

   /* Reed/door slots */
   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      (void)snprintf(name, sizeof(name), "DOOR %u", (unsigned int)(i + 1u));
      sys_row_create(g_sys_list, &g_sys_rows[row], name, 0u);
      row++;
   }

   /* Doorbell cam slots — hidden until first DB frame */
   for (i = 0u; i < (uint8_t)MAX_DOORBELL_CAMS; i++)
   {
      (void)snprintf(name, sizeof(name), "DB%u", (unsigned int)i);
      sys_row_create(g_sys_list, &g_sys_rows[row], name, 1u);
      row++;
   }

   /* Inference camera slots — hidden until first heartbeat received.
    * Shown once g_cam_last_seen[] is stamped by ui_stamp_cam_online(). */
   for (i = 0u; i < (uint8_t)MAX_CAMS; i++)
   {
      (void)snprintf(name, sizeof(name), "CAM%u", (unsigned int)(i + 1u));
      sys_row_create(g_sys_list, &g_sys_rows[row], name, 1u);
      row++;
   }

   lv_obj_add_flag(g_sys_list, LV_OBJ_FLAG_HIDDEN);
}

void system_list_refresh(void)
{
   char     buf[64];
   uint16_t age_s  = AGE_UNKNOWN_VAL;
   uint8_t  row    = 0u;
   uint8_t  i      = 0u;

   /* Row 0 — aggregate TEMP
    * age_s from g_home.temp_age, promoted from slot 0 by
    * ui_set_temp_slot_age() in ui.c. See ui.c note "Slot-to-scalar
    * promotion (2026-06-27)". Dot from g_dev_online[eDEV_TEMP].      */
   age_s = g_home.temp_age;
   if (age_s != AGE_UNKNOWN_VAL)
      (void)snprintf(buf, sizeof(buf), "TEMP   %s  %uC %u%%  A:%us",
                     g_dev_online[eDEV_TEMP] ? "[ON]" : "[--]",
                     g_home.temp, g_home.hum, (unsigned int)age_s);
   else
      (void)snprintf(buf, sizeof(buf), "TEMP   %s  %uC %u%%  A:--",
                     g_dev_online[eDEV_TEMP] ? "[ON]" : "[--]",
                     g_home.temp, g_home.hum);
   sys_row_set(row++, buf, g_dev_online[eDEV_TEMP]);

   /* Row 1 — MOTOR
    * Wire age from g_home.motor_age. Dot from g_dev_online[eDEV_MOTOR]. */
   age_s = g_home.motor_age;
   if (age_s != AGE_UNKNOWN_VAL)
      (void)snprintf(buf, sizeof(buf), "MOTOR  %s  B:%d%%  A:%us",
                     g_dev_online[eDEV_MOTOR] ? "[ON]" : "[--]",
                     g_home.motor_batt, (unsigned int)age_s);
   else
      (void)snprintf(buf, sizeof(buf), "MOTOR  %s  B:%d%%  A:--",
                     g_dev_online[eDEV_MOTOR] ? "[ON]" : "[--]",
                     g_home.motor_batt);
   sys_row_set(row++, buf, g_dev_online[eDEV_MOTOR]);

   /* Row 2 — LIGHT
    * Wire age from g_home.light_age. Dot from g_dev_online[eDEV_LIGHT]. */
   age_s = g_home.light_age;
   if (age_s != AGE_UNKNOWN_VAL)
      (void)snprintf(buf, sizeof(buf), "LIGHT  %s  A:%us",
                     g_dev_online[eDEV_LIGHT] ? "[ON]" : "[--]",
                     (unsigned int)age_s);
   else
      (void)snprintf(buf, sizeof(buf), "LIGHT  %s  A:--",
                     g_dev_online[eDEV_LIGHT] ? "[ON]" : "[--]");
   sys_row_set(row++, buf, g_dev_online[eDEV_LIGHT]);

   /* Row 3 — LOCK
    * Wire age from g_home.lock_age. Dot from g_dev_online[eDEV_LOCK]. */
   age_s = g_home.lock_age;
   if (age_s != AGE_UNKNOWN_VAL)
      (void)snprintf(buf, sizeof(buf), "LOCK   %s  B:%d%%  A:%us",
                     g_dev_online[eDEV_LOCK] ? "[ON]" : "[--]",
                     g_home.lock_batt, (unsigned int)age_s);
   else
      (void)snprintf(buf, sizeof(buf), "LOCK   %s  B:%d%%  A:--",
                     g_dev_online[eDEV_LOCK] ? "[ON]" : "[--]",
                     g_home.lock_batt);
   sys_row_set(row++, buf, g_dev_online[eDEV_LOCK]);

   /* BLE temp slots
    * Wire age from g_home.temp_slot_age[], set by ui_set_temp_slot_age().
    * Dot from g_temp_online[]. Both independent — see ui.c note.        */
   for (i = 0u; i < (uint8_t)MAX_TEMPS; i++)
   {
      if (i < g_temp_count_slots)
      {
         int16_t dg       = g_home.temp_slot_decidegc[i];
         int     deg_int  = (int)(dg / 10);
         int     deg_frac = (int)(dg % 10);
         if (deg_frac < 0) { deg_frac = -deg_frac; }

         age_s = g_home.temp_slot_age[i];
         if (age_s != AGE_UNKNOWN_VAL)
            (void)snprintf(buf, sizeof(buf), "TEMP%-2u %s  %d.%dC  B:%d%%  A:%us",
                           (unsigned int)(i + 1u),
                           g_temp_online[i] ? "[ON]" : "[--]",
                           deg_int, deg_frac,
                           (int)g_home.temp_slot_batt[i],
                           (unsigned int)age_s);
         else
            (void)snprintf(buf, sizeof(buf), "TEMP%-2u %s  %d.%dC  B:%d%%  A:--",
                           (unsigned int)(i + 1u),
                           g_temp_online[i] ? "[ON]" : "[--]",
                           deg_int, deg_frac,
                           (int)g_home.temp_slot_batt[i]);
         lv_obj_clear_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
         sys_row_set(row, buf, g_temp_online[i]);
      }
      else
      {
         lv_obj_add_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
      }
      row++;
   }

   /* PIR slots */
   for (i = 0u; i < (uint8_t)MAX_PIRS; i++)
   {
      if (i < g_pir_count_slots)
      {
         if (g_home.pir_slot_age[i] != AGE_UNKNOWN_VAL)
            (void)snprintf(buf, sizeof(buf), "PIR%-2u  %s  B:%d%%  A:%us",
                           (unsigned int)(i + 1u),
                           g_pir_online[i] ? "[ON]" : "[--]",
                           (int)g_home.pir_slot_batt[i],
                           (unsigned int)g_home.pir_slot_age[i]);
         else
            (void)snprintf(buf, sizeof(buf), "PIR%-2u  %s  B:%d%%  A:--",
                           (unsigned int)(i + 1u),
                           g_pir_online[i] ? "[ON]" : "[--]",
                           (int)g_home.pir_slot_batt[i]);
         lv_obj_clear_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
         sys_row_set(row, buf, g_pir_online[i]);
      }
      else
      {
         lv_obj_add_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
      }
      row++;
   }

   /* Reed/door slots */
   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      if (i < g_reed_count)
      {
         if (g_home.reed_age[i] != AGE_UNKNOWN_VAL)
            (void)snprintf(buf, sizeof(buf), "DOOR%-2u %s  B:%d%%  A:%us",
                           (unsigned int)(i + 1u),
                           g_reed_online[i] ? "[ON]" : "[--]",
                           (int)g_home.reed_batt[i],
                           (unsigned int)g_home.reed_age[i]);
         else
            (void)snprintf(buf, sizeof(buf), "DOOR%-2u %s  B:%d%%  A:--",
                           (unsigned int)(i + 1u),
                           g_reed_online[i] ? "[ON]" : "[--]",
                           (int)g_home.reed_batt[i]);
         lv_obj_clear_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
         sys_row_set(row, buf, g_reed_online[i]);
      }
      else
      {
         lv_obj_add_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
      }
      row++;
   }

   /* Doorbell cam slots */
   for (i = 0u; i < (uint8_t)MAX_DOORBELL_CAMS; i++)
   {
      if (g_home.doorbell_slot_age[i] != AGE_UNKNOWN_VAL)
      {
         (void)snprintf(buf, sizeof(buf), "DB%-2u   %s  A:%us",
                        (unsigned int)i,
                        g_doorbell_online[i] ? "[ON]" : "[--]",
                        (unsigned int)g_home.doorbell_slot_age[i]);
         lv_obj_clear_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
         sys_row_set(row, buf, g_doorbell_online[i]);
      }
      else
      {
         lv_obj_add_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
      }
      row++;
   }

   /* Inference camera slots — hidden until first wire frame received.
    * Visibility gated on cam_age == AGE_UNKNOWN_VAL, matching the
    * pattern used by doorbell, PIR, reed, and temp slot rows.
    * Age from g_home.cam_age[] (wire age_s from hub), not local
    * elapsed — consistent with all other device rows.               */
   for (i = 0u; i < (uint8_t)MAX_CAMS; i++)
   {
      if (g_home.cam_age[i] == AGE_UNKNOWN_VAL)
      {
         lv_obj_add_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
         row++;
         continue;
      }
      (void)snprintf(buf, sizeof(buf), "CAM%u  %s  A:%us",
                     (unsigned int)(i + 1u),
                     g_cam_online[i] ? "[ON]" : "[--]",
                     (unsigned int)g_home.cam_age[i]);
      lv_obj_clear_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
      sys_row_set(row, buf, g_cam_online[i]);
      row++;
   }
}
