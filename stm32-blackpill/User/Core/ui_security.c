/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ui_security.c
 * \author  MichaelLynnCSU
 * \date    2026-06-14
 *
 * \brief   SECURITY view — PIR/reed/lock tile creation, reflow, render.
 ******************************************************************************/

#include "ui_priv.h"

/************************ TILE OBJECTS (externed in ui_priv.h) ****************/

TILE_X g_t_pir_slot[MAX_PIRS];
TILE_X g_t_reed[MAX_REEDS];
TILE_X g_t_lock;

/************************** PUBLIC FUNCTIONS ***********************************/

void create_security_tiles(lv_obj_t *p_scr)
{
   char    title_buf[PIR_TITLE_BUF];
   uint8_t i = 0u;

   for (i = 0u; i < (uint8_t)MAX_PIRS; i++)
   {
      (void)snprintf(title_buf, sizeof(title_buf), "PIR %u", (unsigned int)(i + 1u));
      create_tile(&g_t_pir_slot[i], p_scr, 0, 0, (int)TILE_WIDTH, 60, title_buf);
      lv_obj_add_flag(g_t_pir_slot[i].p_tile, LV_OBJ_FLAG_HIDDEN);
   }

   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      (void)snprintf(title_buf, sizeof(title_buf), "DOOR %u", (unsigned int)(i + 1u));
      create_tile(&g_t_reed[i], p_scr, 0, 0, (int)TILE_WIDTH, 60, title_buf);
      lv_obj_add_flag(g_t_reed[i].p_tile, LV_OBJ_FLAG_HIDDEN);
   }

   create_tile(&g_t_lock, p_scr, 0, 0, (int)FULL_TILE_WIDTH, 60, "SMART LOCK");
   lv_obj_add_flag(g_t_lock.p_tile, LV_OBJ_FLAG_HIDDEN);
}

void reflow_security(void)
{
   int     n_pir          = (int)g_pir_count_slots;
   int     n_reed         = (int)g_reed_count;
   int     pir_row_count  = (n_pir  > 0) ? ((n_pir  + 1) / 2) : 0;
   int     reed_row_count = (n_reed > 0) ? ((n_reed + 1) / 2) : 0;
   int     total_rows     = pir_row_count + reed_row_count + 1;
   int     usable         = (int)CONTENT_BOT - (int)CONTENT_TOP;
   int     gap_total      = (total_rows + 1) * (int)TILE_GAP;
   int     h_budget       = usable - gap_total;
   int     h              = (total_rows > 0) ? (h_budget / total_rows) : 44;
   int     y              = (int)CONTENT_TOP + (int)TILE_GAP;
   int     col            = 0;
   int     row            = 0;
   uint8_t i              = 0u;

   if (h < TILE_H_MIN_SEC) { h = TILE_H_MIN_SEC; }
   if (h > TILE_H_MAX_SEC) { h = TILE_H_MAX_SEC; }

   for (i = 0u; i < (uint8_t)MAX_PIRS; i++)
   {
      if (i < (uint8_t)n_pir)
      {
         col = (int)(i % 2u);
         row = (int)(i / 2u);
         lv_obj_set_pos(g_t_pir_slot[i].p_tile,
                        (0 == col) ? (int)TILE_LEFT_MARGIN : (int)TILE_RIGHT_COL_X,
                        y + row * (h + (int)TILE_GAP));
         lv_obj_set_size(g_t_pir_slot[i].p_tile, (int)TILE_WIDTH, h);
      }
   }
   y += pir_row_count * (h + (int)TILE_GAP);

   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      if (i < (uint8_t)n_reed)
      {
         col = (int)(i % 2u);
         row = (int)(i / 2u);
         lv_obj_set_pos(g_t_reed[i].p_tile,
                        (0 == col) ? (int)TILE_LEFT_MARGIN : (int)TILE_RIGHT_COL_X,
                        y + row * (h + (int)TILE_GAP));
         lv_obj_set_size(g_t_reed[i].p_tile, (int)TILE_WIDTH, h);
      }
   }
   y += reed_row_count * (h + (int)TILE_GAP);

   lv_obj_set_pos(g_t_lock.p_tile,  (int)TILE_LEFT_MARGIN, y);
   lv_obj_set_size(g_t_lock.p_tile, (int)FULL_TILE_WIDTH, h);
}

void ui_security_update(void)
{
   uint8_t i = 0u;

   for (i = 0u; i < g_pir_count_slots && i < (uint8_t)MAX_PIRS; i++)
   {
      lv_label_set_text(g_t_pir_slot[i].p_value,
                        (0u != g_home.pir_slot_occupied[i]) ? "MOTION" : "CLEAR");
      lv_label_set_text(g_t_pir_slot[i].p_sub, "");
      set_status(g_t_pir_slot[i].p_status, g_pir_online[i]);
   }

   for (i = 0u; i < g_reed_count && i < (uint8_t)MAX_REEDS; i++)
   {
      lv_label_set_text(g_t_reed[i].p_value,
                        (0u != g_home.reed_state[i]) ? "OPEN" : "CLOSED");
      lv_label_set_text(g_t_reed[i].p_sub, "");
      set_status(g_t_reed[i].p_status, g_reed_online[i]);
   }

   lv_label_set_text(g_t_lock.p_value,
                     (0u != g_home.lock) ? "LOCKED" : "UNLOCKED");
   lv_label_set_text(g_t_lock.p_sub, "");
   set_status(g_t_lock.p_status, g_dev_online[eDEV_LOCK]);
}
