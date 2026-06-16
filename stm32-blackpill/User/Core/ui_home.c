/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ui_home.c
 * \author  MichaelLynnCSU
 * \date    2026-06-14
 *
 * \brief   HOME view — label creation and per-frame render.
 *
 * \details Renders the HOME view status line, temp/hum, and network status.
 *          Doorbell alert display uses g_home.doorbell / g_home.doorbell_*
 *          fields written by the FSM in ui.c. This module is read-only
 *          with respect to doorbell state — it never clears g_home.doorbell
 *          directly; that is handled by tick_doorbell_timeout() in ui.c.
 ******************************************************************************/

#include "ui_priv.h"

/************************ DEFINED IN ui.c *************************************/

/* Home view label objects — owned here, externed in ui_priv.h */
lv_obj_t *g_home_status_lbl = NULL;
lv_obj_t *g_home_temp_lbl   = NULL;
lv_obj_t *g_home_net_lbl    = NULL;

/************************** PUBLIC FUNCTIONS ***********************************/

void create_home_labels(lv_obj_t *p_scr)
{
   int y = 0;

   if (NULL == p_scr) { return; }

   y = (int)CONTENT_TOP + 20;

   g_home_status_lbl = lv_label_create(p_scr);
   lv_label_set_text(g_home_status_lbl, "HOME SECURE");
   lv_obj_set_style_text_color(g_home_status_lbl, lv_color_hex(0x00FF66u), 0);
   lv_obj_align(g_home_status_lbl, LV_ALIGN_TOP_MID, 0, y);

   y += 40;
   g_home_temp_lbl = lv_label_create(p_scr);
   lv_label_set_text(g_home_temp_lbl, "--C  --%");
   lv_obj_set_style_text_color(g_home_temp_lbl, lv_color_white(), 0);
   lv_obj_align(g_home_temp_lbl, LV_ALIGN_TOP_MID, 0, y);

   y += 40;
   g_home_net_lbl = lv_label_create(p_scr);
   lv_label_set_text(g_home_net_lbl, "CHECKING...");
   lv_obj_set_style_text_color(g_home_net_lbl, lv_color_hex(C_SUB_TEXT), 0);
   lv_obj_align(g_home_net_lbl, LV_ALIGN_TOP_MID, 0, y);
}

void ui_home_update(uint8_t all_online)
{
   char    buf[SNPRINTF_BUF] = {0};
   uint8_t i                 = 0u;
   uint8_t alert             = 0u;

   /* Doorbell alert — render while FSM pending window is open.
    * g_home.doorbell is set by ui_set_doorbell_result() on pressed=1
    * and cleared by tick_doorbell_timeout() after DOORBELL_UI_TIMEOUT_MS.
    * This block never clears it — the FSM owns that transition.           */
   if (0u != g_home.doorbell)
   {
      char db_buf[64];
      if ('\0' == g_home.doorbell_asset[0])
      {
         (void)snprintf(db_buf, sizeof(db_buf),
                        "DOORBELL %u  PENDING...",
                        (unsigned int)g_home.doorbell_device_id);
      }
      else
      {
         (void)snprintf(db_buf, sizeof(db_buf),
                        "DOORBELL %u  %s  %u%%  %.16s",
                        (unsigned int)g_home.doorbell_device_id,
                        g_home.doorbell_person ? "PERSON" : "NO PERSON",
                        (unsigned int)g_home.doorbell_conf_pct,
                        g_home.doorbell_asset);
      }
      lv_label_set_long_mode(g_home_status_lbl, LV_LABEL_LONG_WRAP);
      lv_obj_set_width(g_home_status_lbl, lv_pct(100));
      lv_label_set_text(g_home_status_lbl, db_buf);
      lv_obj_set_style_text_color(g_home_status_lbl, lv_color_hex(0xFFCC00u), 0);
   }
   else
   {
      for (i = 0u; i < g_pir_count_slots && i < (uint8_t)MAX_PIRS; i++)
      {
         if (0u != g_home.pir_slot_occupied[i]) { alert = 1u; break; }
      }
      for (i = 0u; i < g_reed_count && i < (uint8_t)MAX_REEDS; i++)
      {
         if (0u != g_home.reed_state[i]) { alert = 1u; break; }
      }

      if (0u != alert)
      {
         lv_label_set_long_mode(g_home_status_lbl, LV_LABEL_LONG_CLIP);
         lv_obj_set_width(g_home_status_lbl, LV_SIZE_CONTENT);
         lv_label_set_text(g_home_status_lbl, "! ALERT !");
         lv_obj_set_style_text_color(g_home_status_lbl, lv_color_hex(0xFF4444u), 0);
      }
      else
      {
         lv_label_set_long_mode(g_home_status_lbl, LV_LABEL_LONG_CLIP);
         lv_obj_set_width(g_home_status_lbl, LV_SIZE_CONTENT);
         lv_label_set_text(g_home_status_lbl, "HOME SECURE");
         lv_obj_set_style_text_color(g_home_status_lbl, lv_color_hex(0x00FF66u), 0);
      }
   }

   (void)snprintf(buf, sizeof(buf), "%uC  %u%%", g_home.temp, g_home.hum);
   lv_label_set_text(g_home_temp_lbl, buf);

   if (0u != all_online)
   {
      lv_label_set_text(g_home_net_lbl, "ALL DEVICES ONLINE");
      lv_obj_set_style_text_color(g_home_net_lbl, lv_color_hex(C_SUB_TEXT), 0);
   }
   else
   {
      lv_label_set_text(g_home_net_lbl, "DEVICE OFFLINE");
      lv_obj_set_style_text_color(g_home_net_lbl, lv_color_hex(0xFF8800u), 0);
   }
}
