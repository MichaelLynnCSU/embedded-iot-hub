/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ui.c
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   LVGL UI layer — core init, shared state, doorbell FSM, accessors.
 *
 * \details Owns all shared state (g_home, liveness arrays, view state).
 *          View-specific rendering is delegated to:
 *            ui_home.c     — HOME view
 *            ui_security.c — SECURITY view
 *            ui_system.c   — SYSTEM view
 *
 * \note    Doorbell inference FSM (2026-06-14):
 *          g_doorbell_pending + DOORBELL_UI_TIMEOUT_MS implement a
 *          two-phase commit model. pressed=1 opens the transaction;
 *          timeout closes it. Inference fields (person/conf_pct/asset)
 *          are accepted only while pending==1.
 *          See arch spec: doorbell_ui_arch_spec.md
 ******************************************************************************/

#include "ui_priv.h"
#include "log.h"
#include "ili9341.h"
#include "xpt2046.h"
#include "main.h"

/************************ SHARED STATE DEFINITIONS ****************************/
/* All externs declared in ui_priv.h are defined here.                        */

HOME_STATE_X g_home = {
   .pir_slot_batt        = {-1, -1, -1, -1, -1},
   .reed_batt            = {-1, -1, -1, -1, -1, -1},
   .lock_batt            = -1,
   .motor_batt           = -1,
   .temp_slot_batt       = {-1, -1, -1, -1},
   .doorbell             = 0,
   .doorbell_device_id   = 0,
   .doorbell_person      = 0,
   .doorbell_conf_pct    = 0,
   .doorbell_asset       = {0},
   .doorbell_slot_age    = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
   .doorbell_slot_online = {0, 0, 0, 0},
};

uint32_t g_dev_last_seen[eDEV_COUNT];
uint8_t  g_dev_online[eDEV_COUNT];

uint32_t g_reed_last_seen[MAX_REEDS];
uint8_t  g_reed_online[MAX_REEDS];
uint8_t  g_reed_count      = 2u;

uint32_t g_pir_last_seen[MAX_PIRS];
uint8_t  g_pir_online[MAX_PIRS];
uint8_t  g_pir_count_slots = 0u;

uint32_t g_temp_last_seen[MAX_TEMPS];
uint8_t  g_temp_online[MAX_TEMPS];
uint8_t  g_temp_count_slots = 0u;

uint32_t g_doorbell_last_seen[MAX_DOORBELL_CAMS];
uint8_t  g_doorbell_online[MAX_DOORBELL_CAMS];

uint32_t g_cam_last_seen[MAX_CAMS];
uint8_t  g_cam_online[MAX_CAMS];
char     g_cam_buf[MAX_CAMS][32];

UI_VIEW_E g_current_view = eVIEW_HOME;

lv_obj_t  *g_nav_bar         = NULL;
lv_obj_t  *g_nav_lbl[eVIEW_COUNT];

lv_style_t g_style_online;
lv_style_t g_style_offline;

/********************* DOORBELL FSM PRIVATE STATE *****************************/
/* g_doorbell_pending is the sole gate for inference field updates.
 * INVARIANT: only pressed=1 sets it; only timeout clears it.
 * No external caller may read or write these — they are private to ui.c.    */

static uint8_t  g_doorbell_pending    = 0u;
static uint32_t g_doorbell_last_rx_ms = 0u;

/************************ STATIC (PRIVATE) FUNCTIONS **************************/

static void lvgl_flush_cb(lv_disp_drv_t *p_drv,
                          const lv_area_t *p_area,
                          lv_color_t *p_color)
{
   if ((NULL == p_drv) || (NULL == p_area) || (NULL == p_color)) { return; }
   ILI9341_DrawBitmap(p_area->x1, p_area->y1,
                      (int16_t)(p_area->x2 - p_area->x1 + 1),
                      (int16_t)(p_area->y2 - p_area->y1 + 1),
                      (uint16_t *)p_color);
   lv_disp_flush_ready(p_drv);
}

static void touch_read_cb(lv_indev_drv_t *p_drv, lv_indev_data_t *p_data)
{
   (void)p_drv;
   if (XPT2046_IsTouched())
   {
      TouchPoint_t tp = XPT2046_GetTouch();
      p_data->point.x = (lv_coord_t)tp.x;
      p_data->point.y = (lv_coord_t)tp.y;
      p_data->state   = LV_INDEV_STATE_PRESSED;
   }
   else
   {
      p_data->state = LV_INDEV_STATE_RELEASED;
   }
}

static void nav_btn_event_cb(lv_event_t *p_e)
{
   const UI_VIEW_E *p_view = (const UI_VIEW_E *)lv_event_get_user_data(p_e);
   if (NULL != p_view) { ui_set_view(*p_view); }
}

static void init_status_styles(void)
{
   lv_style_init(&g_style_online);
   lv_style_set_bg_opa(&g_style_online,    LV_OPA_COVER);
   lv_style_set_bg_color(&g_style_online,  lv_color_hex(C_STATUS_ON));
   lv_style_set_radius(&g_style_online,    LV_RADIUS_CIRCLE);
   lv_style_set_border_width(&g_style_online,  0);
   lv_style_set_outline_width(&g_style_online, 0);
   lv_style_set_shadow_width(&g_style_online,  0);
   lv_style_set_pad_all(&g_style_online,   0);

   lv_style_init(&g_style_offline);
   lv_style_set_bg_opa(&g_style_offline,   LV_OPA_COVER);
   lv_style_set_bg_color(&g_style_offline, lv_color_hex(C_STATUS_OFF));
   lv_style_set_radius(&g_style_offline,   LV_RADIUS_CIRCLE);
   lv_style_set_border_width(&g_style_offline,  0);
   lv_style_set_outline_width(&g_style_offline, 0);
   lv_style_set_shadow_width(&g_style_offline,  0);
   lv_style_set_pad_all(&g_style_offline,  0);
}

static void create_header(lv_obj_t *p_scr)
{
   lv_obj_t *p_hdr    = NULL;
   lv_obj_t *p_htitle = NULL;

   if (NULL == p_scr) { return; }

   p_hdr = lv_obj_create(p_scr);
   lv_obj_set_pos(p_hdr, 0, 0);
   lv_obj_set_size(p_hdr, (int)DISP_HOR_RES, (int)HDR_HEIGHT);
   lv_obj_set_style_bg_color(p_hdr, lv_color_make(C_HDR_R, C_HDR_G, C_HDR_B), 0);
   lv_obj_set_style_border_width(p_hdr, 0, 0);
   lv_obj_set_style_radius(p_hdr, 0, 0);
   lv_obj_set_style_pad_all(p_hdr, 0, 0);
   lv_obj_clear_flag(p_hdr, LV_OBJ_FLAG_SCROLLABLE);

   p_htitle = lv_label_create(p_hdr);
   lv_label_set_text(p_htitle, "SMART HOME");
   lv_obj_set_style_text_color(p_htitle, lv_color_white(), 0);
   lv_obj_align(p_htitle, LV_ALIGN_CENTER, 0, 0);
}

static void create_nav_bar(lv_obj_t *p_scr)
{
   static const char    *const k_labels[eVIEW_COUNT] = { "HOME", "SEC", "SYS" };
   static const UI_VIEW_E      k_views[eVIEW_COUNT]  =
      { eVIEW_HOME, eVIEW_SECURITY, eVIEW_SYSTEM };

   lv_obj_t *p_btn = NULL;
   lv_obj_t *p_lbl = NULL;
   uint8_t   i     = 0u;

   if (NULL == p_scr) { return; }

   g_nav_bar = lv_obj_create(p_scr);
   lv_obj_set_pos(g_nav_bar, 0, (int)(DISP_VER_RES - NAV_HEIGHT));
   lv_obj_set_size(g_nav_bar, (int)DISP_HOR_RES, (int)NAV_HEIGHT);
   lv_obj_set_style_bg_color(g_nav_bar, lv_color_make(C_HDR_R, C_HDR_G, C_HDR_B), 0);
   lv_obj_set_style_border_width(g_nav_bar, 0, 0);
   lv_obj_set_style_radius(g_nav_bar, 0, 0);
   lv_obj_set_style_pad_all(g_nav_bar, 0, 0);
   lv_obj_clear_flag(g_nav_bar, LV_OBJ_FLAG_SCROLLABLE);

   for (i = 0u; i < (uint8_t)eVIEW_COUNT; i++)
   {
      p_btn = lv_btn_create(g_nav_bar);
      lv_obj_set_pos(p_btn, (int)((uint32_t)i * NAV_ZONE_W), 0);
      lv_obj_set_size(p_btn, (int)NAV_ZONE_W, (int)NAV_HEIGHT);
      lv_obj_set_style_bg_opa(p_btn, LV_OPA_TRANSP, 0);
      lv_obj_set_style_shadow_width(p_btn, 0, 0);
      lv_obj_set_style_border_width(p_btn, 0, 0);
      lv_obj_set_style_pad_all(p_btn, 0, 0);
      lv_obj_set_style_radius(p_btn, 0, 0);
      lv_obj_clear_flag(p_btn, LV_OBJ_FLAG_PRESS_LOCK);
      lv_obj_add_event_cb(p_btn, nav_btn_event_cb, LV_EVENT_CLICKED,
                          (void *)&k_views[i]);

      p_lbl = lv_label_create(p_btn);
      lv_label_set_text(p_lbl, k_labels[i]);
      lv_obj_set_style_text_color(p_lbl, lv_color_hex(C_NAV_IDLE), 0);
      lv_obj_align(p_lbl, LV_ALIGN_CENTER, 0, 0);
      g_nav_lbl[i] = p_lbl;
   }

   nav_highlight(eVIEW_HOME);
}

static void tick_doorbell_timeout(uint32_t now)
{
   /* Sole mechanism for closing the doorbell pending window.
    * Clears g_doorbell_pending and g_home.doorbell so the HOME view
    * stops rendering the alert. No external caller may do this.       */
   if ((1u == g_doorbell_pending) &&
       ((now - g_doorbell_last_rx_ms) >= DOORBELL_UI_TIMEOUT_MS))
   {
      g_doorbell_pending   = 0u;
      g_home.doorbell      = 0u;
      g_home.doorbell_person   = 0u;
      g_home.doorbell_conf_pct = 0u;
      g_home.doorbell_asset[0] = '\0';
   }
}

/************************** SHARED HELPERS (used by ui_*.c) *******************/

void set_status(lv_obj_t *p_box, uint8_t online)
{
   if (NULL == p_box) { return; }
   lv_obj_remove_style_all(p_box);
   lv_obj_add_style(p_box, (0u != online) ? &g_style_online : &g_style_offline, 0);
   lv_obj_set_size(p_box, STATUS_DOT_SIZE, STATUS_DOT_SIZE);
   lv_obj_align(p_box, LV_ALIGN_TOP_RIGHT, STATUS_DOT_OFS_X, STATUS_DOT_OFS_Y);
   lv_obj_invalidate(p_box);
}

void create_tile(TILE_X *p_t, lv_obj_t *p_parent,
                 int x, int y, int w, int h,
                 const char *p_title)
{
   if ((NULL == p_t) || (NULL == p_parent) || (NULL == p_title)) { return; }

   p_t->p_tile = lv_obj_create(p_parent);
   lv_obj_set_pos(p_t->p_tile, x, y);
   lv_obj_set_size(p_t->p_tile, w, h);
   lv_obj_set_style_bg_color(p_t->p_tile, C_TILE, 0);
   lv_obj_set_style_border_width(p_t->p_tile, 0, 0);
   lv_obj_set_style_pad_all(p_t->p_tile, 0, 0);
   lv_obj_clear_flag(p_t->p_tile, LV_OBJ_FLAG_SCROLLABLE);

   p_t->p_label = lv_label_create(p_t->p_tile);
   lv_label_set_text(p_t->p_label, p_title);
   lv_obj_set_pos(p_t->p_label, (int)TILE_LABEL_X, (int)TILE_LABEL_Y);
   lv_obj_set_style_text_color(p_t->p_label, lv_color_white(), 0);

   p_t->p_value = lv_label_create(p_t->p_tile);
   lv_label_set_text(p_t->p_value, "---");
   lv_obj_set_pos(p_t->p_value, (int)TILE_LABEL_X, (int)TILE_VALUE_Y);
   lv_obj_set_style_text_color(p_t->p_value, lv_color_white(), 0);

   p_t->p_sub = lv_label_create(p_t->p_tile);
   lv_label_set_text(p_t->p_sub, "");
   lv_obj_set_pos(p_t->p_sub, (int)TILE_LABEL_X, (int)TILE_SUB_Y);
   lv_obj_set_style_text_color(p_t->p_sub, lv_color_hex(C_SUB_TEXT), 0);

   p_t->p_status = lv_obj_create(p_t->p_tile);
   lv_obj_clear_flag(p_t->p_status, LV_OBJ_FLAG_SCROLLABLE);
   set_status(p_t->p_status, 0u);
}

void nav_highlight(UI_VIEW_E view)
{
   uint8_t i = 0u;
   for (i = 0u; i < (uint8_t)eVIEW_COUNT; i++)
   {
      if (NULL == g_nav_lbl[i]) { continue; }
      lv_obj_set_style_text_color(
         g_nav_lbl[i],
         ((uint8_t)view == i) ? lv_color_hex(C_NAV_ACTIVE) : lv_color_hex(C_NAV_IDLE),
         0);
   }
}

void apply_view(UI_VIEW_E view)
{
   uint8_t i = 0u;

   if (g_home_status_lbl) { lv_obj_add_flag(g_home_status_lbl, LV_OBJ_FLAG_HIDDEN); }
   if (g_home_temp_lbl)   { lv_obj_add_flag(g_home_temp_lbl,   LV_OBJ_FLAG_HIDDEN); }
   if (g_home_net_lbl)    { lv_obj_add_flag(g_home_net_lbl,    LV_OBJ_FLAG_HIDDEN); }
   if (g_sys_list)        { lv_obj_add_flag(g_sys_list,        LV_OBJ_FLAG_HIDDEN); }

   lv_obj_add_flag(g_t_lock.p_tile, LV_OBJ_FLAG_HIDDEN);
   for (i = 0u; i < (uint8_t)MAX_PIRS;  i++) { lv_obj_add_flag(g_t_pir_slot[i].p_tile, LV_OBJ_FLAG_HIDDEN); }
   for (i = 0u; i < (uint8_t)MAX_REEDS; i++) { lv_obj_add_flag(g_t_reed[i].p_tile,     LV_OBJ_FLAG_HIDDEN); }

   switch (view)
   {
      case eVIEW_HOME:
         if (g_home_status_lbl) { lv_obj_clear_flag(g_home_status_lbl, LV_OBJ_FLAG_HIDDEN); }
         if (g_home_temp_lbl)   { lv_obj_clear_flag(g_home_temp_lbl,   LV_OBJ_FLAG_HIDDEN); }
         if (g_home_net_lbl)    { lv_obj_clear_flag(g_home_net_lbl,    LV_OBJ_FLAG_HIDDEN); }
         break;

      case eVIEW_SECURITY:
         for (i = 0u; i < g_pir_count_slots && i < (uint8_t)MAX_PIRS; i++)
         {
            lv_obj_clear_flag(g_t_pir_slot[i].p_tile, LV_OBJ_FLAG_HIDDEN);
         }
         for (i = 0u; i < g_reed_count && i < (uint8_t)MAX_REEDS; i++)
         {
            lv_obj_clear_flag(g_t_reed[i].p_tile, LV_OBJ_FLAG_HIDDEN);
         }
         lv_obj_clear_flag(g_t_lock.p_tile, LV_OBJ_FLAG_HIDDEN);
         break;

      case eVIEW_SYSTEM:
         if (g_sys_list) { lv_obj_clear_flag(g_sys_list, LV_OBJ_FLAG_HIDDEN); }
         break;

      default:
         break;
   }

   nav_highlight(view);
}

/************************** PUBLIC FUNCTIONS ***********************************/

void ui_tick(void)
{
   lv_tick_inc(1u);
}

void ui_create(void)
{
   static lv_disp_draw_buf_t g_draw_buf;
   static lv_color_t         g_buf1[LV_BUF_SIZE];
   static lv_color_t         g_buf2[LV_BUF_SIZE];
   static lv_disp_drv_t      disp_drv;
   static lv_indev_drv_t     indev_drv;

   lv_obj_t *p_scr = NULL;
   uint8_t   i     = 0u;

   for (i = 0u; i < (uint8_t)MAX_CAMS; i++)
   {
      (void)snprintf(g_cam_buf[i], sizeof(g_cam_buf[i]),
                     "CAM%u  [--]", (unsigned int)(i + 1u));
   }

   lv_disp_draw_buf_init(&g_draw_buf, g_buf1, g_buf2, LV_BUF_SIZE);

   lv_disp_drv_init(&disp_drv);
   disp_drv.hor_res  = (lv_coord_t)DISP_HOR_RES;
   disp_drv.ver_res  = (lv_coord_t)DISP_VER_RES;
   disp_drv.flush_cb = lvgl_flush_cb;
   disp_drv.draw_buf = &g_draw_buf;
   (void)lv_disp_drv_register(&disp_drv);

   lv_indev_drv_init(&indev_drv);
   indev_drv.type    = LV_INDEV_TYPE_POINTER;
   indev_drv.read_cb = touch_read_cb;
   (void)lv_indev_drv_register(&indev_drv);

   init_status_styles();

   p_scr = lv_scr_act();
   lv_obj_set_style_bg_color(p_scr, C_BG, 0);
   lv_obj_clear_flag(p_scr, LV_OBJ_FLAG_SCROLLABLE);

   create_header(p_scr);
   create_nav_bar(p_scr);
   create_home_labels(p_scr);
   create_sys_list(p_scr);
   create_security_tiles(p_scr);

   reflow_security();

   g_current_view = eVIEW_HOME;
   apply_view(eVIEW_HOME);
}

void ui_update(void)
{
   uint32_t now        = 0ul;
   uint8_t  i          = 0u;
   uint8_t  all_online = 1u;

   now = HAL_GetTick();

   tick_doorbell_timeout(now);

   for (i = 0u; i < (uint8_t)eDEV_COUNT; i++)
   {
      g_dev_online[i] = ((g_dev_last_seen[i] > 0ul) &&
                         ((now - g_dev_last_seen[i]) < HB_TIMEOUT_MS)) ? 1u : 0u;
      if (0u == g_dev_online[i]) { all_online = 0u; }
   }

   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      g_reed_online[i] = ((g_reed_last_seen[i] > 0ul) &&
                          ((now - g_reed_last_seen[i]) < HB_TIMEOUT_MS)) ? 1u : 0u;
      if (i < g_reed_count && 0u == g_reed_online[i]) { all_online = 0u; }
   }

   for (i = 0u; i < (uint8_t)MAX_PIRS; i++)
   {
      g_pir_online[i] = ((g_pir_last_seen[i] > 0ul) &&
                         ((now - g_pir_last_seen[i]) < HB_TIMEOUT_MS)) ? 1u : 0u;
      if (i < g_pir_count_slots && 0u == g_pir_online[i]) { all_online = 0u; }
   }

   for (i = 0u; i < (uint8_t)MAX_TEMPS; i++)
   {
      g_temp_online[i] = ((g_temp_last_seen[i] > 0ul) &&
                          ((now - g_temp_last_seen[i]) < HB_TIMEOUT_MS)) ? 1u : 0u;
      if (i < g_temp_count_slots && 0u == g_temp_online[i]) { all_online = 0u; }
   }

   for (i = 0u; i < (uint8_t)MAX_DOORBELL_CAMS; i++)
   {
      g_doorbell_online[i] = ((g_doorbell_last_seen[i] > 0ul) &&
                              ((now - g_doorbell_last_seen[i]) < HB_TIMEOUT_MS)) ? 1u : 0u;
      if ((g_home.doorbell_slot_age[i] != AGE_UNKNOWN_VAL) &&
          (0u == g_doorbell_online[i]))
      {
         all_online = 0u;
      }
   }

   for (i = 0u; i < (uint8_t)MAX_CAMS; i++)
   {
      g_cam_online[i] = ((g_cam_last_seen[i] > 0ul) &&
                         ((now - g_cam_last_seen[i]) < HB_TIMEOUT_MS)) ? 1u : 0u;
   }

   switch (g_current_view)
   {
      case eVIEW_HOME:     ui_home_update(all_online);   break;
      case eVIEW_SECURITY: ui_security_update();         break;
      case eVIEW_SYSTEM:   system_list_refresh();        break;
      default:                                           break;
   }
}

void ui_set_view(UI_VIEW_E view)
{
   if (view >= eVIEW_COUNT)    { return; }
   if (view == g_current_view) { return; }

   g_current_view = view;

   if (view == eVIEW_SECURITY) { reflow_security(); }

   apply_view(view);
   ui_update();
}

UI_VIEW_E ui_get_view(void)    { return g_current_view; }
void      ui_poll_touch(void)  { /* touch owned by registered lv_indev driver */ }

void ui_reflow_pir(int n_pir)
{
   if (n_pir < 0)             { n_pir = 0; }
   if (n_pir > (int)MAX_PIRS) { n_pir = (int)MAX_PIRS; }
   g_pir_count_slots = (uint8_t)n_pir;
   reflow_security();
   apply_view(g_current_view);
}

void ui_reflow(int n)
{
   (void)n;
   ui_reflow_pir((int)g_pir_count_slots);
}

void ui_reflow_temp(int n)
{
   if (n < 0)              { n = 0; }
   if (n > (int)MAX_TEMPS) { n = (int)MAX_TEMPS; }
   g_temp_count_slots = (uint8_t)n;
   if (g_current_view == eVIEW_SYSTEM) { system_list_refresh(); }
}

/************************** ACCESSOR FUNCTIONS *********************************/

uint8_t ui_get_reed_count(void)       { return g_reed_count; }
uint8_t ui_get_pir_count_slots(void)  { return g_pir_count_slots; }
uint8_t ui_get_temp_count_slots(void) { return g_temp_count_slots; }

void ui_set_reed_count(uint8_t count)
{
   if (count > (uint8_t)MAX_REEDS) { count = (uint8_t)MAX_REEDS; }
   g_reed_count = count;
}

void ui_set_pir_count_slots(uint8_t count)
{
   if (count > (uint8_t)MAX_PIRS) { count = (uint8_t)MAX_PIRS; }
   g_pir_count_slots = count;
}

void ui_set_temp_count_slots(uint8_t count) { g_temp_count_slots = count; }

void ui_stamp_dev_online(DEVICE_ID_E dev_id, uint32_t tick)
{
   if (dev_id < eDEV_COUNT) { g_dev_last_seen[dev_id] = tick; }
}

void ui_stamp_reed_online(uint8_t slot, uint32_t tick)
{
   if (slot < (uint8_t)MAX_REEDS) { g_reed_last_seen[slot] = tick; }
}

void ui_stamp_pir_online(uint8_t slot, uint32_t tick)
{
   if (slot < (uint8_t)MAX_PIRS) { g_pir_last_seen[slot] = tick; }
}

void ui_stamp_temp_online(uint8_t slot, uint32_t tick)
{
   if (slot < (uint8_t)MAX_TEMPS) { g_temp_last_seen[slot] = tick; }
}

void ui_stamp_doorbell_online(uint8_t slot, uint32_t tick)
{
   if (slot < (uint8_t)MAX_DOORBELL_CAMS) { g_doorbell_last_seen[slot] = tick; }
}

void ui_stamp_cam_online(uint8_t slot, uint32_t tick)
{
   if (slot < (uint8_t)MAX_CAMS) { g_cam_last_seen[slot] = tick; }
}

/* ---- ui_set_cam: config/display only, no liveness authority --------------- */
void ui_set_cam(uint8_t slot)
{
   if (slot >= (uint8_t)MAX_CAMS) { return; }
   /* Refresh the row label text. Online colour is applied in
    * system_list_refresh() using g_cam_online[], which is driven
    * exclusively by ui_stamp_cam_online() + HB_TIMEOUT_MS.           */
   (void)snprintf(g_cam_buf[slot], sizeof(g_cam_buf[slot]),
                  "CAM%u", (unsigned int)(slot + 1u));
}

void ui_set_temp(uint8_t val)         { g_home.temp         = val; }
void ui_set_hum(uint8_t val)          { g_home.hum          = val; }
void ui_set_pir_count(uint32_t val)   { g_home.pir_count    = val; }
void ui_set_pir_batt(uint8_t val)     { g_home.pir_batt     = val; }
void ui_set_pir_occupied(uint8_t val) { g_home.pir_occupied = val; }
void ui_set_light(uint8_t val)        { g_home.light        = val; }
void ui_set_lock(uint8_t val)         { g_home.lock         = val; }
void ui_set_lock_batt(int8_t val)     { g_home.lock_batt    = val; }
void ui_set_motor(uint8_t val)        { g_home.motor        = val; }
void ui_set_motor_batt(int val)       { g_home.motor_batt   = val; }

/* ---- Doorbell FSM entry points ------------------------------------------- */

void ui_set_doorbell(uint8_t pressed, uint8_t device_id)
{
   /* Legacy shim — zero inference fields */
   ui_set_doorbell_result(pressed, device_id, 0u, 0u, "");
}

void ui_set_doorbell_result(uint8_t pressed, uint8_t device_id,
                            uint8_t person,  uint8_t conf_pct,
                            const char *p_asset)
{
   uint32_t now = HAL_GetTick();

   if (0u != pressed)
   {
      /* Open (or re-open) the pending window and snapshot trigger fields.
       * Clear asset so ui_home_update() shows PENDING... until the
       * five-field inference frame arrives.                            */
      g_doorbell_pending        = 1u;
      g_doorbell_last_rx_ms     = now;
      g_home.doorbell           = 1u;
      g_home.doorbell_device_id = device_id;
      g_home.doorbell_asset[0]  = '\0';
      g_home.doorbell_person    = 0u;
      g_home.doorbell_conf_pct  = 0u;
   }

   if (1u == g_doorbell_pending)
   {
      /* Late inference binding — accepted while event window is open.
       * press→inference delta is 1-4 s by design: the ESP32 fires
       * pressed=1 immediately on GPIO, but the BBB inference pipeline
       * (capture → model → SHM → UART) adds latency. person/conf_pct/
       * asset will be zero on the first DOORBELL frame and arrive on
       * a subsequent pressed=0 frame within the pending window.       */
      g_home.doorbell_person   = person;
      g_home.doorbell_conf_pct = conf_pct;
      if (NULL != p_asset)
      {
         (void)strncpy(g_home.doorbell_asset, p_asset,
                       sizeof(g_home.doorbell_asset) - 1u);
         g_home.doorbell_asset[sizeof(g_home.doorbell_asset) - 1u] = '\0';
      }
   }
}

void ui_set_doorbell_slot_age(uint8_t slot, uint16_t age_s)
{
   if (slot < (uint8_t)MAX_DOORBELL_CAMS)
   {
      g_home.doorbell_slot_age[slot] = age_s;
   }
}

void ui_set_doorbell_slot_online(uint8_t slot, uint8_t online)
{
   if (slot < (uint8_t)MAX_DOORBELL_CAMS)
   {
      g_home.doorbell_slot_online[slot] = online;
   }
}

void ui_set_pir_slot_count(uint8_t slot, uint32_t val)
{
   if (slot < (uint8_t)MAX_PIRS) { g_home.pir_slot_count[slot] = val; }
}

void ui_set_pir_slot_batt(uint8_t slot, int8_t batt)
{
   if (slot < (uint8_t)MAX_PIRS) { g_home.pir_slot_batt[slot] = batt; }
}

void ui_set_pir_slot_age(uint8_t slot, uint16_t age)
{
   if (slot < (uint8_t)MAX_PIRS) { g_home.pir_slot_age[slot] = age; }
}

void ui_set_pir_slot_occupied(uint8_t slot, uint8_t val)
{
   if (slot < (uint8_t)MAX_PIRS) { g_home.pir_slot_occupied[slot] = val; }
}

void ui_set_reed_state(uint8_t slot, uint8_t state)
{
   if (slot < (uint8_t)MAX_REEDS) { g_home.reed_state[slot] = state; }
}

void ui_set_reed_batt(uint8_t slot, int8_t batt)
{
   if (slot < (uint8_t)MAX_REEDS) { g_home.reed_batt[slot] = batt; }
}

void ui_set_reed_age(uint8_t slot, uint16_t age)
{
   if (slot < (uint8_t)MAX_REEDS) { g_home.reed_age[slot] = age; }
}

void ui_set_temp_slot_decidegc(uint8_t slot, int16_t val)
{
   if (slot < (uint8_t)MAX_TEMPS) { g_home.temp_slot_decidegc[slot] = val; }
}

void ui_set_temp_slot_batt(uint8_t slot, int8_t batt)
{
   if (slot < (uint8_t)MAX_TEMPS) { g_home.temp_slot_batt[slot] = batt; }
}

void ui_set_temp_slot_age(uint8_t slot, uint16_t age)
{
   if (slot < (uint8_t)MAX_TEMPS) { g_home.temp_slot_age[slot] = age; }
}

uint8_t ui_get_dev_online(DEVICE_ID_E dev_id)
{
   if (dev_id >= eDEV_COUNT) { return 0u; }
   return g_dev_online[dev_id];
}
