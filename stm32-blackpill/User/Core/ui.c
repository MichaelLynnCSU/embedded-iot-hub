/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ui.c
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   LVGL UI layer implementation for the Smart Home Dashboard.
 *
 * \details Manages the LVGL display driver registration, tile creation,
 *          adaptive layout reflow, live sensor-state rendering, and
 *          device online/offline status dots. All HomeState mutations are
 *          performed through the accessor functions declared in ui.h so
 *          that the parser and main modules remain decoupled from the
 *          internal data structures.
 *
 * \note    Multi-view architecture (2026-06-01):
 *          Three views replace the single-screen compressed layout.
 *          HOME     — status headline, temperature, network summary.
 *          SECURITY — PIR slot tiles + reed tiles + lock tile.
 *          SYSTEM   — scrollable list, one row per device.
 *          Navigation: 28px nav bar at bottom, three tap zones.
 *          Touch polling: ui_poll_touch() called from main loop.
 *
 * \note    SYSTEM view (2026-06-01):
 *          Replaced tile grid with a scrollable LVGL label list.
 *          One lv_label_t per device, updated via lv_label_set_text().
 *          No reflow math — LVGL flex column handles layout.
 *          Scales to 24+ devices without modification.
 *
 * \note    SECURITY view (2026-06-01):
 *          Tiles now show operational state only: OPEN/CLOSED/MOTION/LOCKED.
 *          Battery and age moved to SYSTEM view.
 *
 * \note    System list color fix (2026-06-01):
 *          sys_row_set() helper added. All system list rows now render
 *          green when online and red when offline. Unicode dot glyphs
 *          replaced with ASCII [ON]/[--] — default LVGL font does not
 *          include U+25CF/U+25CB codepoints.
 *
 * \note    BLE temp slots in SYSTEM view (2026-06-03):
 *          SYS_ROW_COUNT increased by MAX_TEMPS.
 *          create_sys_list() adds one row per temp slot after fixed rows.
 *          system_list_refresh() renders TEMP1/TEMP2 with decidegc, online
 *          status, and battery. Inactive slots are hidden.
 *          ui_reflow_temp() stub promoted to update g_temp_count_slots and
 *          refresh the system list when slot count changes.
 *
 * \note    Doorbell liveness (2026-06-09):
 *          g_doorbell_last_seen[MAX_DOORBELL_CAMS] and
 *          g_doorbell_online[MAX_DOORBELL_CAMS] added — mirror PIR/temp
 *          per-slot online tracking. HOME_STATE_X gains
 *          doorbell_slot_age[MAX_DOORBELL_CAMS] and
 *          doorbell_slot_online[MAX_DOORBELL_CAMS].
 *          SYS_ROW_COUNT increased by MAX_DOORBELL_CAMS.
 *          system_list_refresh() renders DB0..DB3 rows after reed rows.
 *          ui_set_doorbell_slot_age(), ui_set_doorbell_slot_online(),
 *          ui_stamp_doorbell_online() added.
 ******************************************************************************/

#include "ui.h"
#include "log.h"
#include "ili9341.h"
#include "xpt2046.h"
#include "lvgl.h"
#include <stdio.h>
#include <string.h>

/******************************** CONSTANTS ***********************************/

#define C_BG              lv_color_hex(0x101010u)
#define C_TILE            lv_color_hex(0x2C2C2Cu)
#define C_HDR_R           0x01u
#define C_HDR_G           0x19u
#define C_HDR_B           0x31u
#define C_STATUS_ON       0x00FF00u
#define C_STATUS_OFF      0x500000u
#define C_SUB_TEXT        0xAAAAAAu
#define C_NAV_ACTIVE      0x00CC44u
#define C_NAV_IDLE        0x888888u
#define C_ROW_ONLINE      0x00FF66u
#define C_ROW_OFFLINE     0xFF4444u

#define LV_BUF_LINES      20u
#define LV_BUF_SIZE       (240u * LV_BUF_LINES)
#define DISP_HOR_RES      240u
#define DISP_VER_RES      320u
#define STATUS_DOT_SIZE     8u
#define STATUS_DOT_OFS_X   -6
#define STATUS_DOT_OFS_Y    5
#define TILE_LABEL_X        8u
#define TILE_LABEL_Y        4u
#define TILE_VALUE_Y       22u
#define TILE_SUB_Y         42u
#define REED_TITLE_BUF     16u
#define PIR_TITLE_BUF      16u
#define SNPRINTF_BUF       48u
#define AGE_UNKNOWN_VAL    0xFFFFu

#define TILE_H_MIN_SEC     36
#define TILE_H_MAX_SEC     72

#define TOUCH_DEBOUNCE_MS  150ul

/* Fixed rows: TEMP(aggregate), MOTOR, LIGHT, LOCK = 4
 * Then one row per BLE temp slot, PIR slot, reed slot, doorbell cam. */
#define SYS_ROW_COUNT  (4u + MAX_TEMPS + MAX_PIRS + MAX_REEDS + MAX_DOORBELL_CAMS)

/************************** STRUCTURE DATA TYPES ******************************/

typedef struct _TILE_X
{
   lv_obj_t *p_tile;
   lv_obj_t *p_label;
   lv_obj_t *p_value;
   lv_obj_t *p_sub;
   lv_obj_t *p_status;
} TILE_X;

typedef struct _HOME_STATE_X
{
   uint8_t  temp;
   uint8_t  hum;
   uint32_t pir_count;
   uint8_t  pir_batt;
   uint8_t  pir_occupied;
   uint32_t pir_slot_count[MAX_PIRS];
   int8_t   pir_slot_batt[MAX_PIRS];
   uint16_t pir_slot_age[MAX_PIRS];
   uint8_t  pir_slot_occupied[MAX_PIRS];
   int16_t  temp_slot_decidegc[MAX_TEMPS];
   int8_t   temp_slot_batt[MAX_TEMPS];
   uint16_t temp_slot_age[MAX_TEMPS];
   uint8_t  reed_state[MAX_REEDS];
   int8_t   reed_batt[MAX_REEDS];
   uint16_t reed_age[MAX_REEDS];
   uint8_t  light;
   uint8_t  lock;
   int8_t   lock_batt;
   uint8_t  motor;
   int      motor_batt;
   uint8_t  doorbell;
   uint8_t  doorbell_device_id;
   uint16_t doorbell_slot_age[MAX_DOORBELL_CAMS];    /*!< per-cam age seconds */
   uint8_t  doorbell_slot_online[MAX_DOORBELL_CAMS]; /*!< per-cam online flag */
} HOME_STATE_X;

/************************ STATIC (PRIVATE) DATA *****************************/

static HOME_STATE_X g_home = {
   .pir_slot_batt          = {-1, -1, -1, -1, -1},
   .reed_batt              = {-1, -1, -1, -1, -1, -1},
   .lock_batt              = -1,
   .motor_batt             = -1,
   .temp_slot_batt         = {-1, -1, -1, -1},
   .doorbell               = 0,
   .doorbell_device_id     = 0,
   .doorbell_slot_age      = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF},
   .doorbell_slot_online   = {0, 0, 0, 0},
};

static uint32_t g_dev_last_seen[eDEV_COUNT];
static uint8_t  g_dev_online[eDEV_COUNT];
static uint32_t g_reed_last_seen[MAX_REEDS];
static uint8_t  g_reed_online[MAX_REEDS];
static uint32_t g_pir_last_seen[MAX_PIRS];
static uint8_t  g_pir_online[MAX_PIRS];
static uint8_t  g_reed_count       = 2u;
static uint8_t  g_pir_count_slots  = 0u;
static uint32_t g_temp_last_seen[MAX_TEMPS];
static uint8_t  g_temp_online[MAX_TEMPS];
static uint8_t  g_temp_count_slots = 0u;

/* Doorbell per-cam online tracking — mirrors PIR/temp pattern */
static uint32_t g_doorbell_last_seen[MAX_DOORBELL_CAMS];
static uint8_t  g_doorbell_online[MAX_DOORBELL_CAMS];

static lv_disp_draw_buf_t g_draw_buf;
static lv_color_t         g_buf1[LV_BUF_SIZE];
static lv_color_t         g_buf2[LV_BUF_SIZE];

static lv_style_t g_style_online;
static lv_style_t g_style_offline;

/* ---- Security view tiles ------------------------------------------------ */
static TILE_X g_t_pir_slot[MAX_PIRS];
static TILE_X g_t_reed[MAX_REEDS];
static TILE_X g_t_lock;

/* ---- View state --------------------------------------------------------- */
static UI_VIEW_E  g_current_view  = eVIEW_HOME;

/* ---- Nav bar ------------------------------------------------------------ */
static lv_obj_t  *g_nav_bar            = NULL;
static lv_obj_t  *g_nav_lbl[eVIEW_COUNT];

/* ---- Home view labels --------------------------------------------------- */
static lv_obj_t  *g_home_status_lbl = NULL;
static lv_obj_t  *g_home_temp_lbl   = NULL;
static lv_obj_t  *g_home_net_lbl    = NULL;

/* ---- System view list --------------------------------------------------- */
static lv_obj_t  *g_sys_list                = NULL;
static lv_obj_t  *g_sys_rows[SYS_ROW_COUNT];

static const char *const k_nav_labels[eVIEW_COUNT] = { "HOME", "SEC", "SYS" };

/************************ STATIC (PRIVATE) FUNCTIONS **************************/
static void nav_btn_event_cb(lv_event_t *p_e);

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

static void set_status(lv_obj_t *p_box, uint8_t online)
{
   if (NULL == p_box) { return; }
   lv_obj_remove_style_all(p_box);
   lv_obj_add_style(p_box, (0u != online) ? &g_style_online : &g_style_offline, 0);
   lv_obj_set_size(p_box, STATUS_DOT_SIZE, STATUS_DOT_SIZE);
   lv_obj_align(p_box, LV_ALIGN_TOP_RIGHT, STATUS_DOT_OFS_X, STATUS_DOT_OFS_Y);
   lv_obj_invalidate(p_box);
}

static void create_tile(TILE_X *p_t, lv_obj_t *p_parent,
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

static void init_status_styles(void)
{
   lv_style_init(&g_style_online);
   lv_style_set_bg_opa(&g_style_online, LV_OPA_COVER);
   lv_style_set_bg_color(&g_style_online, lv_color_hex(C_STATUS_ON));
   lv_style_set_radius(&g_style_online, LV_RADIUS_CIRCLE);
   lv_style_set_border_width(&g_style_online, 0);
   lv_style_set_outline_width(&g_style_online, 0);
   lv_style_set_shadow_width(&g_style_online, 0);
   lv_style_set_pad_all(&g_style_online, 0);

   lv_style_init(&g_style_offline);
   lv_style_set_bg_opa(&g_style_offline, LV_OPA_COVER);
   lv_style_set_bg_color(&g_style_offline, lv_color_hex(C_STATUS_OFF));
   lv_style_set_radius(&g_style_offline, LV_RADIUS_CIRCLE);
   lv_style_set_border_width(&g_style_offline, 0);
   lv_style_set_outline_width(&g_style_offline, 0);
   lv_style_set_shadow_width(&g_style_offline, 0);
   lv_style_set_pad_all(&g_style_offline, 0);
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

static void nav_highlight(UI_VIEW_E view)
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

static void create_nav_bar(lv_obj_t *p_scr)
{
   static const UI_VIEW_E k_views[eVIEW_COUNT] =
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
      lv_label_set_text(p_lbl, k_nav_labels[i]);
      lv_obj_set_style_text_color(p_lbl, lv_color_hex(C_NAV_IDLE), 0);
      lv_obj_align(p_lbl, LV_ALIGN_CENTER, 0, 0);
      g_nav_lbl[i] = p_lbl;
   }

   nav_highlight(eVIEW_HOME);
}

static void create_home_labels(lv_obj_t *p_scr)
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

static void create_sys_list(lv_obj_t *p_scr)
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

   /* Fixed rows: aggregate TEMP, MOTOR, LIGHT, LOCK */
   const char *const fixed[4] = { "TEMP", "MOTOR", "LIGHT", "LOCK" };
   for (i = 0u; i < 4u; i++)
   {
      sys_row_create(g_sys_list, &g_sys_rows[row], fixed[i], 0u);
      row++;
   }

   /* One row per BLE temp slot — hidden until temp_count arrives */
   for (i = 0u; i < (uint8_t)MAX_TEMPS; i++)
   {
      (void)snprintf(name, sizeof(name), "TEMP %u", (unsigned int)(i + 1u));
      sys_row_create(g_sys_list, &g_sys_rows[row], name, 1u);
      row++;
   }

   /* One row per PIR slot */
   for (i = 0u; i < (uint8_t)MAX_PIRS; i++)
   {
      (void)snprintf(name, sizeof(name), "PIR %u", (unsigned int)(i + 1u));
      sys_row_create(g_sys_list, &g_sys_rows[row], name, 0u);
      row++;
   }

   /* One row per reed/door slot */
   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      (void)snprintf(name, sizeof(name), "DOOR %u", (unsigned int)(i + 1u));
      sys_row_create(g_sys_list, &g_sys_rows[row], name, 0u);
      row++;
   }

   /* One row per doorbell cam — hidden until first DB frame arrives */
   for (i = 0u; i < (uint8_t)MAX_DOORBELL_CAMS; i++)
   {
      (void)snprintf(name, sizeof(name), "CAM %u", (unsigned int)i);
      sys_row_create(g_sys_list, &g_sys_rows[row], name, 1u);
      row++;
   }

   lv_obj_add_flag(g_sys_list, LV_OBJ_FLAG_HIDDEN);
}

static void apply_view(UI_VIEW_E view)
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

static void reflow_security(void)
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
   int     y              = 0;
   int     col            = 0;
   int     row            = 0;
   uint8_t i              = 0u;

   if (h < TILE_H_MIN_SEC) { h = TILE_H_MIN_SEC; }
   if (h > TILE_H_MAX_SEC) { h = TILE_H_MAX_SEC; }

   y = (int)CONTENT_TOP + (int)TILE_GAP;

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

static void sys_row_set(uint8_t row, const char *p_buf, uint8_t online)
{
   if (NULL == g_sys_rows[row]) { return; }
   lv_label_set_text(g_sys_rows[row], p_buf);
   lv_obj_set_style_text_color(g_sys_rows[row],
      (0u != online) ? lv_color_hex(C_ROW_ONLINE) : lv_color_hex(C_ROW_OFFLINE),
      0);
}

/**
 * \brief  Refresh SYSTEM list rows from current state.
 *
 * \details Row layout:
 *            [0]                                   TEMP (aggregate)
 *            [1]                                   MOTOR
 *            [2]                                   LIGHT
 *            [3]                                   LOCK
 *            [4 .. 4+MAX_TEMPS-1]                  BLE temp slots
 *            [4+MAX_TEMPS .. +MAX_PIRS-1]           PIR slots
 *            [4+MAX_TEMPS+MAX_PIRS .. +MAX_REEDS-1] Reed/door slots
 *            [4+MAX_TEMPS+MAX_PIRS+MAX_REEDS .. +MAX_DOORBELL_CAMS-1]
 *                                                  Doorbell cams
 */
static void system_list_refresh(void)
{
   char    buf[48];
   uint8_t row = 0u;
   uint8_t i   = 0u;

   /* Row 0 — aggregate TEMP */
   (void)snprintf(buf, sizeof(buf), "TEMP   %s  %uC %u%%",
                  g_dev_online[eDEV_TEMP] ? "[ON]" : "[--]",
                  g_home.temp, g_home.hum);
   sys_row_set(row++, buf, g_dev_online[eDEV_TEMP]);

   /* Row 1 — MOTOR */
   (void)snprintf(buf, sizeof(buf), "MOTOR  %s  B:%d%%",
                  g_dev_online[eDEV_MOTOR] ? "[ON]" : "[--]",
                  g_home.motor_batt);
   sys_row_set(row++, buf, g_dev_online[eDEV_MOTOR]);

   /* Row 2 — LIGHT */
   (void)snprintf(buf, sizeof(buf), "LIGHT  %s",
                  g_dev_online[eDEV_LIGHT] ? "[ON]" : "[--]");
   sys_row_set(row++, buf, g_dev_online[eDEV_LIGHT]);

   /* Row 3 — LOCK */
   (void)snprintf(buf, sizeof(buf), "LOCK   %s  B:%d%%",
                  g_dev_online[eDEV_LOCK] ? "[ON]" : "[--]",
                  g_home.lock_batt);
   sys_row_set(row++, buf, g_dev_online[eDEV_LOCK]);

   /* BLE temp slots */
   for (i = 0u; i < (uint8_t)MAX_TEMPS; i++)
   {
      if (i < g_temp_count_slots)
      {
         int16_t dg        = g_home.temp_slot_decidegc[i];
         int     deg_int   = (int)(dg / 10);
         int     deg_frac  = (int)(dg % 10);
         if (deg_frac < 0) { deg_frac = -deg_frac; }

         (void)snprintf(buf, sizeof(buf), "TEMP%-2u %s  %d.%dC  B:%d%%",
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
         {
            (void)snprintf(buf, sizeof(buf), "PIR%-2u  %s  B:%d%%  A:%us",
                           (unsigned int)(i + 1u),
                           g_pir_online[i] ? "[ON]" : "[--]",
                           (int)g_home.pir_slot_batt[i],
                           (unsigned int)g_home.pir_slot_age[i]);
         }
         else
         {
            (void)snprintf(buf, sizeof(buf), "PIR%-2u  %s  B:%d%%  A:--",
                           (unsigned int)(i + 1u),
                           g_pir_online[i] ? "[ON]" : "[--]",
                           (int)g_home.pir_slot_batt[i]);
         }
         lv_obj_clear_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
         sys_row_set(row, buf, g_pir_online[i]);
      }
      else
      {
         lv_obj_add_flag(g_sys_rows[row], LV_OBJ_FLAG_HIDDEN);
      }
      row++;
   }

   /* Reed slots */
   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      if (i < g_reed_count)
      {
         (void)snprintf(buf, sizeof(buf), "DOOR%-2u %s  B:%d%%",
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

   /* Doorbell cams — show all MAX_DOORBELL_CAMS; hide if never seen */
   for (i = 0u; i < (uint8_t)MAX_DOORBELL_CAMS; i++)
   {
      if (g_home.doorbell_slot_age[i] != AGE_UNKNOWN_VAL)
      {
         (void)snprintf(buf, sizeof(buf), "CAM%-2u  %s  A:%us",
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
}

static void touch_read_cb(lv_indev_drv_t *p_drv, lv_indev_data_t *p_data)
{
   (void)p_drv;
   if (XPT2046_IsTouched())
   {
      TouchPoint_t tp  = XPT2046_GetTouch();
      p_data->point.x  = (lv_coord_t)tp.x;
      p_data->point.y  = (lv_coord_t)tp.y;
      p_data->state    = LV_INDEV_STATE_PRESSED;
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


/************************** PUBLIC FUNCTIONS ***********************************/

void ui_tick(void)
{
   lv_tick_inc(1u);
}

void ui_create(void)
{
   static lv_disp_drv_t disp_drv;
   lv_obj_t            *p_scr     = NULL;
   char                 title_buf[PIR_TITLE_BUF];
   uint8_t              i         = 0u;

   lv_disp_draw_buf_init(&g_draw_buf, g_buf1, g_buf2, LV_BUF_SIZE);

   lv_disp_drv_init(&disp_drv);
   disp_drv.hor_res  = (lv_coord_t)DISP_HOR_RES;
   disp_drv.ver_res  = (lv_coord_t)DISP_VER_RES;
   disp_drv.flush_cb = lvgl_flush_cb;
   disp_drv.draw_buf = &g_draw_buf;
   (void)lv_disp_drv_register(&disp_drv);

   (void)lv_disp_drv_register(&disp_drv);

   static lv_indev_drv_t indev_drv;
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

   reflow_security();

   g_current_view = eVIEW_HOME;
   apply_view(eVIEW_HOME);
}

void ui_update(void)
{
   char     buf[SNPRINTF_BUF] = {0};
   uint32_t now               = 0ul;
   uint8_t  i                 = 0u;
   uint8_t  all_online        = 1u;

   now = HAL_GetTick();

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
      /* Only flag all_online=0 if cam has ever checked in (age != UNKNOWN) */
      if ((g_home.doorbell_slot_age[i] != AGE_UNKNOWN_VAL) &&
          (0u == g_doorbell_online[i]))
      {
         all_online = 0u;
      }
   }

   /* HOME */
   if (g_current_view == eVIEW_HOME)
   {
      uint8_t alert    = 0u;
      uint8_t doorbell = g_home.doorbell;

      if (0u != doorbell)
      {
         char db_buf[32];
         snprintf(db_buf, sizeof(db_buf), "DOORBELL %d",
                  g_home.doorbell_device_id);
         lv_label_set_text(g_home_status_lbl, db_buf);
         lv_obj_set_style_text_color(g_home_status_lbl,
                                     lv_color_hex(0xFFCC00u), 0);
         g_home.doorbell = 0u;
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
            lv_label_set_text(g_home_status_lbl, "! ALERT !");
            lv_obj_set_style_text_color(g_home_status_lbl,
                                        lv_color_hex(0xFF4444u), 0);
         }
         else
         {
            lv_label_set_text(g_home_status_lbl, "HOME SECURE");
            lv_obj_set_style_text_color(g_home_status_lbl,
                                        lv_color_hex(0x00FF66u), 0);
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

      return;
   }

   /* SECURITY */
   if (g_current_view == eVIEW_SECURITY)
   {
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

      return;
   }

   /* SYSTEM */
   if (g_current_view == eVIEW_SYSTEM)
   {
      system_list_refresh();
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

UI_VIEW_E ui_get_view(void)
{
   return g_current_view;
}

void ui_poll_touch(void)
{
   /* No-op: touch is now owned by the registered lv_indev driver.
    * LVGL calls touch_read_cb() from lv_task_handler() each tick.
    * Nav taps are dispatched via nav_btn_event_cb(). */
}

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

   if (g_current_view == eVIEW_SYSTEM)
   {
      system_list_refresh();
   }
}

/* ---- Accessor functions ------------------------------------------------- */

uint8_t ui_get_reed_count(void) { return g_reed_count; }

void ui_set_reed_count(uint8_t count)
{
   if (count > (uint8_t)MAX_REEDS) { count = (uint8_t)MAX_REEDS; }
   g_reed_count = count;
}

uint8_t ui_get_pir_count_slots(void) { return g_pir_count_slots; }

void ui_set_pir_count_slots(uint8_t count)
{
   if (count > (uint8_t)MAX_PIRS) { count = (uint8_t)MAX_PIRS; }
   g_pir_count_slots = count;
}

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

void ui_set_temp(uint8_t val)            { g_home.temp         = val; }
void ui_set_hum(uint8_t val)             { g_home.hum          = val; }
void ui_set_pir_count(uint32_t val)      { g_home.pir_count    = val; }
void ui_set_pir_batt(uint8_t val)        { g_home.pir_batt     = val; }
void ui_set_pir_occupied(uint8_t val)    { g_home.pir_occupied = val; }
void ui_set_light(uint8_t val)           { g_home.light        = val; }
void ui_set_lock(uint8_t val)            { g_home.lock         = val; }
void ui_set_lock_batt(int8_t val)        { g_home.lock_batt    = val; }
void ui_set_motor(uint8_t val)           { g_home.motor        = val; }
void ui_set_motor_batt(int val)          { g_home.motor_batt   = val; }
void ui_set_doorbell(uint8_t pressed, uint8_t device_id)
{
   g_home.doorbell           = pressed;
   g_home.doorbell_device_id = device_id;
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

void ui_set_temp_count_slots(uint8_t count) { g_temp_count_slots = count; }
uint8_t ui_get_temp_count_slots(void)       { return g_temp_count_slots; }

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
