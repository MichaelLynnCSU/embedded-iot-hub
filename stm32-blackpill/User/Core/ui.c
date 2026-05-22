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
 *          Feature switches:
 *            UI_DEBUG_REFLOW — logs reflow dimensions to USB CDC when defined.
 *
 * \note    Occupied indicator (2026-04-30):
 *          pir_occupied added to HOME_STATE_X. PIR tile value label now
 *          prefixes count with "OCC" or "---" to show sliding-window
 *          occupancy state received via OCC: UART frame from BeagleBone.
 *
 * \note    Per-slot PIR tiles (2026-05-XX):
 *          pir_slot_count/batt/age arrays added to HOME_STATE_X.
 *          g_t_pir_slot[] tile array mirrors the reed DR<n> pattern.
 *          ui_reflow_pir() positions and shows/hides PIR slot tiles in a
 *          two-column grid inserted between the MOTION row and DOOR rows.
 *          ui_stamp_pir_online() drives per-slot online dots via
 *          g_pir_last_seen[]/g_pir_online[].
 *
 * \note    Per-slot OCC (2026-05-20):
 *          pir_slot_occupied[] added to HOME_STATE_X. Per-slot PIR tile
 *          value now prefixes count with "OCC" or "---" driven by
 *          OCC<1-4>:n frames from BeagleBone via ui_set_pir_slot_occupied().
 *
 * \note    PIR tiles match reed pattern (2026-05-21):
 *          PIR slot tiles now render identically to reed tiles:
 *          - value label: "OCC" or "---" + count
 *          - sub label:   "BATT X%  AGE Xs" — age visible on tile
 *          - online dot:  driven by g_pir_last_seen[] / HB_TIMEOUT_MS
 *          - parse_pir_slot() guards AGE_UNKNOWN before stamping online
 *            (fixes PIR 2 dot stuck red when age=0xFFFF on first frame)
 *
 * \note    Motor tile moved to top row (2026-05-21):
 *          Motor tile now shares the top row with temperature (right column).
 *          Removed from the bottom full-width position to prevent tile
 *          overlap on crowded layouts.
 ******************************************************************************/

#include "ui.h"
#include "log.h"
#include "ili9341.h"
#include "lvgl.h"
#include <stdio.h>
#include <string.h>

/******************************** CONSTANTS ***********************************/

#define C_BG              lv_color_hex(0x101010u)  /**< Screen background colour    */
#define C_TILE            lv_color_hex(0x2C2C2Cu)  /**< Tile background colour      */
#define C_HDR_R           0x01u                    /**< Header colour red component  */
#define C_HDR_G           0x19u                    /**< Header colour green component*/
#define C_HDR_B           0x31u                    /**< Header colour blue component */
#define C_STATUS_ON       0x00FF00u                /**< Online dot colour (green)    */
#define C_STATUS_OFF      0x500000u                /**< Offline dot colour (dark red)*/
#define C_SUB_TEXT        0xAAAAAAu                /**< Sub-label text colour        */

#define LV_BUF_LINES      20u                      /**< LVGL draw buffer line count  */
#define LV_BUF_SIZE       (240u * LV_BUF_LINES)    /**< LVGL draw buffer size        */
#define DISP_HOR_RES      240u                      /**< Display horizontal resolution*/
#define DISP_VER_RES      320u                      /**< Display vertical resolution  */
#define HDR_HEIGHT         28u                      /**< Header bar height in pixels  */
#define HDR_WIDTH         240u                      /**< Header bar width in pixels   */
#define STATUS_DOT_SIZE     8u                      /**< Online indicator dot size     */
#define STATUS_DOT_OFS_X   -6                       /**< Dot x-offset from tile edge  */
#define STATUS_DOT_OFS_Y    5                       /**< Dot y-offset from tile edge  */
#define TILE_LABEL_X        8u                      /**< Tile label x-position        */
#define TILE_LABEL_Y        4u                      /**< Tile title y-position        */
#define TILE_VALUE_Y       22u                      /**< Tile value label y-position  */
#define TILE_SUB_Y         42u                      /**< Tile sub-label y-position    */
#define REED_TITLE_BUF     16u                      /**< Buffer for "DOOR N" strings  */
#define PIR_TITLE_BUF      16u                      /**< Buffer for "PIR N" strings   */
#define SNPRINTF_BUF       48u                      /**< General formatting buffer    */
#define AGE_UNKNOWN_VAL    0xFFFFu                  /**< Sentinel for unknown age     */

/************************** STRUCTURE DATA TYPES ******************************/

/**
 * \brief Holds the LVGL objects that make up one dashboard tile.
 */
typedef struct _TILE_X
{
   lv_obj_t *p_tile;   /*!< Tile container object    */
   lv_obj_t *p_label;  /*!< Title label              */
   lv_obj_t *p_value;  /*!< Primary value label      */
   lv_obj_t *p_sub;    /*!< Sub-text label (battery) */
   lv_obj_t *p_status; /*!< Online/offline dot       */
} TILE_X;

/**
 * \brief Aggregated sensor state for the whole dashboard.
 */
typedef struct _HOME_STATE_X
{
   uint8_t  temp;                         /*!< Temperature in degrees C             */
   uint8_t  hum;                          /*!< Relative humidity percent            */
   uint32_t pir_count;                    /*!< Cumulative PIR trigger count (agg.)  */
   uint8_t  pir_batt;                     /*!< PIR battery percent (aggregate)      */
   uint8_t  pir_occupied;                 /*!< 1=occupied, 0=empty (sliding window) */
   uint32_t pir_slot_count[MAX_PIRS];     /*!< Per-slot motion counts               */
   int8_t   pir_slot_batt[MAX_PIRS];      /*!< Per-slot battery percent, -1=unknown */
   uint16_t pir_slot_age[MAX_PIRS];       /*!< Per-slot BLE age in seconds          */
   uint8_t  pir_slot_occupied[MAX_PIRS];  /*!< Per-slot occupancy flag — 2026-05-20 */
   uint8_t  reed_state[MAX_REEDS];        /*!< Reed state: 0=closed, 1=open         */
   int8_t   reed_batt[MAX_REEDS];         /*!< Reed battery percent, -1=unknown     */
   uint16_t reed_age[MAX_REEDS];          /*!< Reed BLE age in seconds              */
   uint8_t  light;                        /*!< Smart light: 0=off, 1=on             */
   uint8_t  lock;                         /*!< Smart lock: 0=unlocked, 1=locked     */
   int8_t   lock_batt;                    /*!< Lock battery percent, -1=unknown     */
   uint8_t  motor;                        /*!< Motor: 0=off, 1=cooling, 2=heating   */
   int      motor_batt;                   /*!< Motor supply mV, -1=unknown          */
} HOME_STATE_X;

/**
 * \brief Per-reed-count tile height parameters.
 */
typedef struct _TILE_LAYOUT_X
{
   int top_h;  /*!< Height of top-row tiles (temp, motor) */
   int pir_h;  /*!< Height of each per-slot PIR tile      */
   int reed_h; /*!< Height of each reed tile              */
   int bot_h;  /*!< Height of bottom-row tiles            */
   int mot_h;  /*!< Unused — kept for ABI stability       */
} TILE_LAYOUT_X;

/************************ STATIC (PRIVATE) DATA *****************************/

/**< All fields zero-init; battery fields initialised to -1 (unknown). */
static HOME_STATE_X g_home = {
   .pir_slot_batt = {-1, -1, -1, -1},
   .reed_batt     = {-1, -1, -1, -1, -1, -1},
   .lock_batt     = -1,
   .motor_batt    = -1,
};

static uint32_t g_dev_last_seen[eDEV_COUNT];      /**< Tick of last device message    */
static uint8_t  g_dev_online[eDEV_COUNT];         /**< Computed online flag per dev    */
static uint32_t g_reed_last_seen[MAX_REEDS];      /**< Tick of last reed message       */
static uint8_t  g_reed_online[MAX_REEDS];         /**< Computed online flag per reed   */
static uint32_t g_pir_last_seen[MAX_PIRS];        /**< Tick of last per-slot PIR msg   */
static uint8_t  g_pir_online[MAX_PIRS];           /**< Computed online flag per PIR    */
static uint8_t  g_reed_count      = 2u;           /**< Active reed count               */
static uint8_t  g_pir_count_slots = 0u;           /**< Active per-slot PIR count       */

static lv_disp_draw_buf_t g_draw_buf;             /**< LVGL draw buffer descriptor     */
static lv_color_t         g_buf1[LV_BUF_SIZE];    /**< Primary render buffer           */
static lv_color_t         g_buf2[LV_BUF_SIZE];    /**< Secondary render buffer         */

static lv_style_t g_style_online;   /**< Style for online status dot   */
static lv_style_t g_style_offline;  /**< Style for offline status dot  */

static TILE_X g_t_temp;                    /**< Temperature tile              */
static TILE_X g_t_pir_slot[MAX_PIRS];      /**< Per-slot PIR tiles            */
static TILE_X g_t_reed[MAX_REEDS];         /**< Reed sensor tiles             */
static TILE_X g_t_light;                   /**< Smart light tile              */
static TILE_X g_t_lock;                    /**< Smart lock tile               */
static TILE_X g_t_motor;                   /**< Cooling/heating motor tile    */

/************************ STATIC (PRIVATE) FUNCTIONS **************************/

/**
 * \brief  LVGL flush callback — transfers the render buffer to the ILI9341.
 */
static void lvgl_flush_cb(lv_disp_drv_t *p_drv,
                          const lv_area_t *p_area,
                          lv_color_t *p_color)
{
   if ((NULL == p_drv) || (NULL == p_area) || (NULL == p_color))
   {
      return;
   }

   ILI9341_DrawBitmap(p_area->x1, p_area->y1,
                      (int16_t)(p_area->x2 - p_area->x1 + 1),
                      (int16_t)(p_area->y2 - p_area->y1 + 1),
                      (uint16_t *)p_color);
   lv_disp_flush_ready(p_drv);
}

/**
 * \brief  Apply the online or offline style to a status dot object.
 */
static void set_status(lv_obj_t *p_box, uint8_t online)
{
   if (NULL == p_box)
   {
      return;
   }

   lv_obj_remove_style_all(p_box);
   lv_obj_add_style(p_box, (0u != online) ? &g_style_online : &g_style_offline, 0);
   lv_obj_set_size(p_box, STATUS_DOT_SIZE, STATUS_DOT_SIZE);
   lv_obj_align(p_box, LV_ALIGN_TOP_RIGHT, STATUS_DOT_OFS_X, STATUS_DOT_OFS_Y);
   lv_obj_invalidate(p_box);
}

/**
 * \brief  Create one dashboard tile with title, value, sub, and status labels.
 */
static void create_tile(TILE_X *p_t, lv_obj_t *p_parent,
                        int x, int y, int w, int h,
                        const char *p_title)
{
   if ((NULL == p_t) || (NULL == p_parent) || (NULL == p_title))
   {
      return;
   }

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

/**
 * \brief  Return tile height parameters for a given total active tile count.
 *
 * \param  n_reed - Active reed count.
 * \param  n_pir  - Active per-slot PIR count.
 *
 * \return TILE_LAYOUT_X - Struct with height fields.
 *
 * \note   Motor now shares the top row with temperature; its row is no
 *         longer counted separately.
 */
static TILE_LAYOUT_X get_layout(int n_reed, int n_pir)
{
   TILE_LAYOUT_X layout  = {0, 0, 0, 0, 0};
   int           rows    = 0;

   rows  = 1;                       /* temp / motor row (shared) */
   rows += (n_pir > 0) ? ((n_pir + 1) / 2) : 0;
   rows += (n_reed + 1) / 2;
   rows += 1;                       /* light / lock row          */

   if (rows <= 4)
   {
      layout.top_h  = 60;
      layout.pir_h  = 60;
      layout.reed_h = 60;
      layout.bot_h  = 60;
      layout.mot_h  = 60;
   }
   else if (rows <= 6)
   {
      layout.top_h  = 54;
      layout.pir_h  = 50;
      layout.reed_h = 50;
      layout.bot_h  = 50;
      layout.mot_h  = 50;
   }
   else
   {
      layout.top_h  = 46;
      layout.pir_h  = 44;
      layout.reed_h = 44;
      layout.bot_h  = 44;
      layout.mot_h  = 46;
   }

   return layout;
}

/**
 * \brief  Initialise status-dot LVGL styles (called once during ui_create).
 */
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

/**
 * \brief  Create the header bar at the top of the dashboard screen.
 */
static void create_header(lv_obj_t *p_scr)
{
   lv_obj_t *p_hdr    = NULL;
   lv_obj_t *p_htitle = NULL;

   if (NULL == p_scr)
   {
      return;
   }

   p_hdr = lv_obj_create(p_scr);
   lv_obj_set_pos(p_hdr, 0, 0);
   lv_obj_set_size(p_hdr, (int)HDR_WIDTH, (int)HDR_HEIGHT);
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

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  Increment the LVGL tick counter by 1 ms (call from SysTick ISR).
 */
void ui_tick(void)
{
   lv_tick_inc(1u);
}

/**
 * \brief  Reposition all tiles for a given active reed count.
 *
 * \param  n - Number of active reed sensors (clamped to 1..MAX_REEDS).
 */
void ui_reflow(int n)
{
   ui_reflow_pir((int)g_pir_count_slots);
   (void)n;
}

/**
 * \brief  Full layout reflow for both per-slot PIR and reed sections.
 *
 * \param  n_pir - Number of active per-slot PIR tiles (0..MAX_PIRS).
 *
 * \note   Top row: temperature (left column), motor (right column).
 *         PIR slot rows follow, then reed rows, then light/lock.
 *         Motor is no longer placed at the bottom.
 */
void ui_reflow_pir(int n_pir)
{
   TILE_LAYOUT_X layout    = {0, 0, 0, 0, 0};
   int           n_reed    = (int)g_reed_count;
   int           y         = 0;
   int           col       = 0;
   int           row       = 0;
   int           pir_rows  = 0;
   int           reed_rows = 0;
   uint8_t       i         = 0u;

   if (n_pir < 0)              { n_pir  = 0; }
   if (n_pir > (int)MAX_PIRS)  { n_pir  = (int)MAX_PIRS; }
   if (n_reed < 1)             { n_reed = 1; }
   if (n_reed > (int)MAX_REEDS){ n_reed = (int)MAX_REEDS; }

   layout = get_layout(n_reed, n_pir);
   y      = (int)HDR_HEIGHT + (int)TILE_GAP;

   /* Row 0 — temperature (left) + motor (right) */
   lv_obj_set_pos(g_t_temp.p_tile,  (int)TILE_LEFT_MARGIN, y);
   lv_obj_set_size(g_t_temp.p_tile, (int)TILE_WIDTH, layout.top_h);
   lv_obj_set_pos(g_t_motor.p_tile, (int)TILE_RIGHT_COL_X, y);
   lv_obj_set_size(g_t_motor.p_tile,(int)TILE_WIDTH, layout.top_h);
   y += layout.top_h + (int)TILE_GAP;

   /* PIR slot rows — two columns, mirrors reed layout */
   for (i = 0u; i < (uint8_t)MAX_PIRS; i++)
   {
      if (i < (uint8_t)n_pir)
      {
         col = (int)(i % 2u);
         row = (int)(i / 2u);
         lv_obj_set_pos(g_t_pir_slot[i].p_tile,
                        (0 == col) ? (int)TILE_LEFT_MARGIN : (int)TILE_RIGHT_COL_X,
                        y + row * (layout.pir_h + (int)TILE_GAP));
         lv_obj_set_size(g_t_pir_slot[i].p_tile, (int)TILE_WIDTH, layout.pir_h);
         lv_obj_clear_flag(g_t_pir_slot[i].p_tile, LV_OBJ_FLAG_HIDDEN);
      }
      else
      {
         lv_obj_add_flag(g_t_pir_slot[i].p_tile, LV_OBJ_FLAG_HIDDEN);
      }
   }

   pir_rows = (n_pir > 0) ? ((n_pir + 1) / 2) : 0;
   y += pir_rows * (layout.pir_h + (int)TILE_GAP);

   /* Reed rows */
   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      if (i < (uint8_t)n_reed)
      {
         col = (int)(i % 2u);
         row = (int)(i / 2u);
         lv_obj_set_pos(g_t_reed[i].p_tile,
                        (0 == col) ? (int)TILE_LEFT_MARGIN : (int)TILE_RIGHT_COL_X,
                        y + row * (layout.reed_h + (int)TILE_GAP));
         lv_obj_set_size(g_t_reed[i].p_tile, (int)TILE_WIDTH, layout.reed_h);
         lv_obj_clear_flag(g_t_reed[i].p_tile, LV_OBJ_FLAG_HIDDEN);
      }
      else
      {
         lv_obj_add_flag(g_t_reed[i].p_tile, LV_OBJ_FLAG_HIDDEN);
      }
   }

   reed_rows = (n_reed + 1) / 2;
   y += reed_rows * (layout.reed_h + (int)TILE_GAP);

   /* Light / lock row */
   lv_obj_set_pos(g_t_light.p_tile, (int)TILE_LEFT_MARGIN, y);
   lv_obj_set_pos(g_t_lock.p_tile,  (int)TILE_RIGHT_COL_X, y);
   lv_obj_set_size(g_t_light.p_tile, (int)TILE_WIDTH, layout.bot_h);
   lv_obj_set_size(g_t_lock.p_tile,  (int)TILE_WIDTH, layout.bot_h);
}

/**
 * \brief  Initialise LVGL display driver and create all dashboard tiles.
 */
void ui_create(void)
{
   static lv_disp_drv_t disp_drv;
   lv_obj_t            *p_scr  = NULL;
   char                 title_buf[PIR_TITLE_BUF];
   uint8_t              i      = 0u;

   lv_disp_draw_buf_init(&g_draw_buf, g_buf1, g_buf2, LV_BUF_SIZE);

   lv_disp_drv_init(&disp_drv);
   disp_drv.hor_res  = (lv_coord_t)DISP_HOR_RES;
   disp_drv.ver_res  = (lv_coord_t)DISP_VER_RES;
   disp_drv.flush_cb = lvgl_flush_cb;
   disp_drv.draw_buf = &g_draw_buf;
   (void)lv_disp_drv_register(&disp_drv);

   init_status_styles();

   p_scr = lv_scr_act();
   lv_obj_set_style_bg_color(p_scr, C_BG, 0);
   lv_obj_clear_flag(p_scr, LV_OBJ_FLAG_SCROLLABLE);

   create_header(p_scr);

   create_tile(&g_t_temp,  p_scr, 0, 0, (int)TILE_WIDTH, 60, "TEMPERATURE");
   create_tile(&g_t_motor, p_scr, 0, 0, (int)TILE_WIDTH, 60, "MOTOR");

   /* Per-slot PIR tiles — all hidden until PIR_COUNT:n arrives */
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

   create_tile(&g_t_light, p_scr, 0, 0, (int)TILE_WIDTH, 60, "SMART LIGHT");
   create_tile(&g_t_lock,  p_scr, 0, 0, (int)TILE_WIDTH, 60, "SMART LOCK");

   ui_reflow_pir(0);
}

/**
 * \brief  Refresh all tile labels and status dots from the current sensor state.
 *
 * \details PIR slot tiles now mirror reed tiles exactly:
 *          - value: occupancy prefix + count
 *          - sub:   "BATT X%  AGE Xs" — age visible on tile
 *          - dot:   driven by g_pir_last_seen[] heartbeat timeout
 */
void ui_update(void)
{
   char     buf[SNPRINTF_BUF] = {0};
   uint32_t now               = 0ul;
   uint8_t  i                 = 0u;

   now = HAL_GetTick();

   /* Recompute online flags */
   for (i = 0u; i < (uint8_t)eDEV_COUNT; i++)
   {
      g_dev_online[i] = ((g_dev_last_seen[i] > 0ul) &&
                         ((now - g_dev_last_seen[i]) < HB_TIMEOUT_MS)) ? 1u : 0u;
   }

   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      g_reed_online[i] = ((g_reed_last_seen[i] > 0ul) &&
                          ((now - g_reed_last_seen[i]) < HB_TIMEOUT_MS)) ? 1u : 0u;
   }

   for (i = 0u; i < (uint8_t)MAX_PIRS; i++)
   {
      g_pir_online[i] = ((g_pir_last_seen[i] > 0ul) &&
                         ((now - g_pir_last_seen[i]) < HB_TIMEOUT_MS)) ? 1u : 0u;
   }

   /* Temperature tile */
   (void)snprintf(buf, sizeof(buf), "%uC  %u%%", g_home.temp, g_home.hum);
   lv_label_set_text(g_t_temp.p_value, buf);
   set_status(g_t_temp.p_status, g_dev_online[eDEV_TEMP]);

   /* Per-slot PIR tiles — identical pattern to reed tiles */
   for (i = 0u; (i < g_pir_count_slots) && (i < (uint8_t)MAX_PIRS); i++)
   {
      /* Value: occupancy prefix + cumulative count */
      (void)snprintf(buf, sizeof(buf), "%s %lu",
                     (0u != g_home.pir_slot_occupied[i]) ? "OCC" : "---",
                     (unsigned long)g_home.pir_slot_count[i]);
      lv_label_set_text(g_t_pir_slot[i].p_value, buf);

      /* Sub: battery and age — mirrors reed batt/age sub label */
      if (g_home.pir_slot_batt[i] >= 0)
      {
         if (g_home.pir_slot_age[i] != AGE_UNKNOWN_VAL)
         {
            (void)snprintf(buf, sizeof(buf), "B:%d%%  A:%us",
                           g_home.pir_slot_batt[i],
                           g_home.pir_slot_age[i]);
         }
         else
         {
            (void)snprintf(buf, sizeof(buf), "B:%d%%  A:--",
                           g_home.pir_slot_batt[i]);
         }
         lv_label_set_text(g_t_pir_slot[i].p_sub, buf);
      }
      else
      {
         lv_label_set_text(g_t_pir_slot[i].p_sub, "");
      }

      /* Online dot — same heartbeat logic as reed */
      set_status(g_t_pir_slot[i].p_status, g_pir_online[i]);
   }

   /* Reed tiles */
   for (i = 0u; (i < g_reed_count) && (i < (uint8_t)MAX_REEDS); i++)
   {
      lv_label_set_text(g_t_reed[i].p_value,
                        (0u != g_home.reed_state[i]) ? "OPEN" : "CLOSED");
      if (g_home.reed_batt[i] >= 0)
      {
         (void)snprintf(buf, sizeof(buf), "BATT %d%%", g_home.reed_batt[i]);
         lv_label_set_text(g_t_reed[i].p_sub, buf);
      }
      else
      {
         lv_label_set_text(g_t_reed[i].p_sub, "");
      }
      set_status(g_t_reed[i].p_status, g_reed_online[i]);
   }

   /* Light tile */
   lv_label_set_text(g_t_light.p_value, (0u != g_home.light) ? "ON" : "OFF");
   set_status(g_t_light.p_status, g_dev_online[eDEV_LIGHT]);

   /* Lock tile */
   lv_label_set_text(g_t_lock.p_value, (0u != g_home.lock) ? "LOCKED" : "UNLOCKED");
   if (g_home.lock_batt >= 0)
   {
      (void)snprintf(buf, sizeof(buf), "BATT %d%%", g_home.lock_batt);
      lv_label_set_text(g_t_lock.p_sub, buf);
   }
   set_status(g_t_lock.p_status, g_dev_online[eDEV_LOCK]);

   /* Motor tile */
   if (0u == g_home.motor)
   {
      lv_label_set_text(g_t_motor.p_value, "OFF");
   }
   else if (1u == g_home.motor)
   {
      lv_label_set_text(g_t_motor.p_value, "COOLING");
   }
   else
   {
      lv_label_set_text(g_t_motor.p_value, "HEATING");
   }

   if (g_home.motor_batt > 0)
   {
      (void)snprintf(buf, sizeof(buf), "BATT %d%%", g_home.motor_batt);
      lv_label_set_text(g_t_motor.p_sub, buf);
   }
   else
   {
      lv_label_set_text(g_t_motor.p_sub, "BATT --");
   }
   set_status(g_t_motor.p_status, g_dev_online[eDEV_MOTOR]);
}

/* ---- Accessor functions ---- */

uint8_t ui_get_reed_count(void)
{
   return g_reed_count;
}

void ui_set_reed_count(uint8_t count)
{
   if (count > (uint8_t)MAX_REEDS) { count = (uint8_t)MAX_REEDS; }
   g_reed_count = count;
}

uint8_t ui_get_pir_count_slots(void)
{
   return g_pir_count_slots;
}

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

/**
 * \brief  Record the current tick as the last-seen time for a per-slot PIR.
 *
 * \param  slot - Zero-based PIR index (must be < MAX_PIRS).
 * \param  tick - HAL_GetTick() value to record.
 *
 * \note   Only called from parse_pir_slot() when age < BLE_AGE_THRESHOLD_S
 *         and age != AGE_UNKNOWN (0xFFFF). Guards against first-frame
 *         AGE_UNKNOWN keeping the dot red permanently.
 */
void ui_stamp_pir_online(uint8_t slot, uint32_t tick)
{
   if (slot < (uint8_t)MAX_PIRS) { g_pir_last_seen[slot] = tick; }
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

/**
 * \brief  Set per-slot PIR motion count.
 *
 * \param  slot - Zero-based PIR index.
 * \param  val  - Motion event count.
 */
void ui_set_pir_slot_count(uint8_t slot, uint32_t val)
{
   if (slot < (uint8_t)MAX_PIRS) { g_home.pir_slot_count[slot] = val; }
}

/**
 * \brief  Set per-slot PIR battery percent.
 *
 * \param  slot - Zero-based PIR index.
 * \param  batt - Battery percent (0-100), or -1 if unknown.
 */
void ui_set_pir_slot_batt(uint8_t slot, int8_t batt)
{
   if (slot < (uint8_t)MAX_PIRS) { g_home.pir_slot_batt[slot] = batt; }
}

/**
 * \brief  Set per-slot PIR BLE advertisement age in seconds.
 *
 * \param  slot - Zero-based PIR index.
 * \param  age  - Age in seconds since last BLE advertisement.
 */
void ui_set_pir_slot_age(uint8_t slot, uint16_t age)
{
   if (slot < (uint8_t)MAX_PIRS) { g_home.pir_slot_age[slot] = age; }
}

/**
 * \brief  Set per-slot PIR occupancy flag.
 *
 * \param  slot - Zero-based PIR index.
 * \param  val  - 1 = occupied, 0 = empty.
 */
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

uint8_t ui_get_dev_online(DEVICE_ID_E dev_id)
{
   if (dev_id >= eDEV_COUNT) { return 0u; }
   return g_dev_online[dev_id];
}
