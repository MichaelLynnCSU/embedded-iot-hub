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
#define SNPRINTF_BUF       32u                      /**< General formatting buffer    */

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
   uint8_t  temp;                  /*!< Temperature in degrees C             */
   uint8_t  hum;                   /*!< Relative humidity percent            */
   uint32_t pir_count;             /*!< Cumulative PIR trigger count         */
   uint8_t  pir_batt;              /*!< PIR battery percent                  */
   uint8_t  reed_state[MAX_REEDS]; /*!< Reed state: 0=closed, 1=open         */
   int8_t   reed_batt[MAX_REEDS];  /*!< Reed battery percent, -1=unknown     */
   uint16_t reed_age[MAX_REEDS];   /*!< Reed BLE age in seconds              */
   uint8_t  light;                 /*!< Smart light: 0=off, 1=on             */
   uint8_t  lock;                  /*!< Smart lock: 0=unlocked, 1=locked     */
   int8_t   lock_batt;             /*!< Lock battery percent, -1=unknown     */
   uint8_t  motor;                 /*!< Motor: 0=off, 1=cooling, 2=heating   */
   int      motor_batt;            /*!< motor supply mV, -1=unknown          */
} HOME_STATE_X;

/**
 * \brief Per-reed-count tile height parameters.
 */
typedef struct _TILE_LAYOUT_X
{
   int top_h;  /*!< Height of top-row tiles (temp, pir) */
   int reed_h; /*!< Height of each reed tile            */
   int bot_h;  /*!< Height of bottom-row tiles          */
   int mot_h;  /*!< Height of the motor tile            */
} TILE_LAYOUT_X;

/************************** STATIC (PRIVATE) DATA *****************************/

/**< All fields zero-init; populated from ESP32 STATE message on first receipt.
 *   reed_batt initialised to -1 (unknown) per field semantics. */
static HOME_STATE_X g_home = {
   .reed_batt = {-1, -1, -1, -1, -1, -1},
   .lock_batt = -1,
   .motor_batt = -1,
};

static uint32_t g_dev_last_seen[eDEV_COUNT];   /**< Tick of last device message   */
static uint8_t  g_dev_online[eDEV_COUNT];      /**< Computed online flag per dev   */
static uint32_t g_reed_last_seen[MAX_REEDS];   /**< Tick of last reed message      */
static uint8_t  g_reed_online[MAX_REEDS];      /**< Computed online flag per reed  */
static uint8_t  g_reed_count      = 2u;        /**< Active reed count              */

static lv_disp_draw_buf_t g_draw_buf;          /**< LVGL draw buffer descriptor    */
static lv_color_t         g_buf1[LV_BUF_SIZE]; /**< Primary render buffer          */
static lv_color_t         g_buf2[LV_BUF_SIZE]; /**< Secondary render buffer        */

static lv_style_t g_style_online;  /**< Style for online status dot  */
static lv_style_t g_style_offline; /**< Style for offline status dot */

static TILE_X g_t_temp;                /**< Temperature tile              */
static TILE_X g_t_pir;                 /**< PIR motion tile               */
static TILE_X g_t_reed[MAX_REEDS];     /**< Reed sensor tiles             */
static TILE_X g_t_light;               /**< Smart light tile              */
static TILE_X g_t_lock;                /**< Smart lock tile               */
static TILE_X g_t_motor;              /**< Cooling/heating motor tile    */

/************************ STATIC (PRIVATE) FUNCTIONS **************************/

/**
 * \brief  LVGL flush callback — transfers the render buffer to the ILI9341.
 *
 * \param  p_drv     - LVGL display driver handle.
 * \param  p_area    - Screen rectangle being updated.
 * \param  p_color   - Pixel colour buffer.
 *
 * \return void
 *
 * \author MichaelLynnCSU
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
 *
 * \param  p_box   - LVGL object representing the status dot.
 * \param  online  - Non-zero if the device is online.
 *
 * \return void
 *
 * \author MichaelLynnCSU
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
 *
 * \param  p_t      - Pointer to the TILE_X struct to populate.
 * \param  p_parent - Parent LVGL screen object.
 * \param  x        - Tile x-position in pixels.
 * \param  y        - Tile y-position in pixels.
 * \param  w        - Tile width in pixels.
 * \param  h        - Tile height in pixels.
 * \param  p_title  - Null-terminated title string.
 *
 * \return void
 *
 * \author MichaelLynnCSU
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
 * \brief  Return tile height parameters for a given reed sensor count.
 *
 * \param  n - Active reed sensor count (1..MAX_REEDS).
 *
 * \return TILE_LAYOUT_X - Struct with top_h, reed_h, bot_h, mot_h fields.
 *
 * \details Layout verified to fit within 320px for n = 1..6:
 *            n=1-2: 264px  n=3-4: 294px  n=5-6: 312px
 *
 * \author MichaelLynnCSU
 */
static TILE_LAYOUT_X get_layout(int n)
{
   TILE_LAYOUT_X layout = {0, 0, 0, 0}; /**< Computed layout parameters */

   if (n <= 2)
   {
      layout.top_h  = 60;
      layout.reed_h = 60;
      layout.bot_h  = 60;
      layout.mot_h  = 60;
   }
   else if (n <= 4)
   {
      layout.top_h  = 54;
      layout.reed_h = 52;
      layout.bot_h  = 52;
      layout.mot_h  = 52;
   }
   else
   {
      layout.top_h  = 50;
      layout.reed_h = 44;
      layout.bot_h  = 44;
      layout.mot_h  = 50;
   }

   return layout;
}

/**
 * \brief  Initialise status-dot LVGL styles (called once during ui_create).
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
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
 *
 * \param  p_scr - Active LVGL screen object.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void create_header(lv_obj_t *p_scr)
{
   lv_obj_t *p_hdr    = NULL; /**< Header bar container */
   lv_obj_t *p_htitle = NULL; /**< Header title label   */

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
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_tick(void)
{
   lv_tick_inc(1u);
}

/**
 * \brief  Reposition all tiles for a given active reed count.
 *
 * \param  n - Number of active reed sensors (clamped to 1..MAX_REEDS).
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_reflow(int n)
{
   TILE_LAYOUT_X layout  = {0, 0, 0, 0}; /**< Height parameters for this count */
   int           y       = 0;             /**< Running y-cursor                 */
   int           col     = 0;             /**< Column index (0=left, 1=right)   */
   int           row     = 0;             /**< Row index within reed section    */
   int           reed_rows = 0;           /**< Number of reed tile rows         */
   uint8_t       i       = 0u;            /**< Reed tile loop index             */

   if (n < 1)
   {
      n = 1;
   }

   if (n > (int)MAX_REEDS)
   {
      n = (int)MAX_REEDS;
   }

   layout = get_layout(n);
   y      = (int)HDR_HEIGHT + (int)TILE_GAP;

   lv_obj_set_pos(g_t_temp.p_tile, (int)TILE_LEFT_MARGIN, y);
   lv_obj_set_pos(g_t_pir.p_tile,  (int)TILE_RIGHT_COL_X, y);
   lv_obj_set_size(g_t_temp.p_tile, (int)TILE_WIDTH, layout.top_h);
   lv_obj_set_size(g_t_pir.p_tile,  (int)TILE_WIDTH, layout.top_h);
   y += layout.top_h + (int)TILE_GAP;

   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      if (i < (uint8_t)n)
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

   reed_rows = (n + 1) / 2;
   y += reed_rows * (layout.reed_h + (int)TILE_GAP);

   lv_obj_set_pos(g_t_light.p_tile, (int)TILE_LEFT_MARGIN, y);
   lv_obj_set_pos(g_t_lock.p_tile,  (int)TILE_RIGHT_COL_X, y);
   lv_obj_set_size(g_t_light.p_tile, (int)TILE_WIDTH, layout.bot_h);
   lv_obj_set_size(g_t_lock.p_tile,  (int)TILE_WIDTH, layout.bot_h);
   y += layout.bot_h + (int)TILE_GAP;

   lv_obj_set_pos(g_t_motor.p_tile, (int)TILE_LEFT_MARGIN, y);
   lv_obj_set_size(g_t_motor.p_tile, (int)FULL_TILE_WIDTH, layout.mot_h);
}

/**
 * \brief  Initialise LVGL display driver and create all dashboard tiles.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_create(void)
{
   static lv_disp_drv_t disp_drv;       /**< LVGL display driver (static lifetime) */
   lv_obj_t            *p_scr  = NULL;  /**< Active screen object                  */
   char                 reed_title[REED_TITLE_BUF]; /**< Formatted reed tile title  */
   uint8_t              i      = 0u;    /**< Reed tile creation loop index          */

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
   create_tile(&g_t_pir,   p_scr, 0, 0, (int)TILE_WIDTH, 60, "MOTION");

   for (i = 0u; i < (uint8_t)MAX_REEDS; i++)
   {
      (void)snprintf(reed_title, sizeof(reed_title), "DOOR %u", (unsigned int)(i + 1u));
      create_tile(&g_t_reed[i], p_scr, 0, 0, (int)TILE_WIDTH, 60, reed_title);
      lv_obj_add_flag(g_t_reed[i].p_tile, LV_OBJ_FLAG_HIDDEN);
   }

   create_tile(&g_t_light, p_scr, 0, 0, (int)TILE_WIDTH,      60, "SMART LIGHT");
   create_tile(&g_t_lock,  p_scr, 0, 0, (int)TILE_WIDTH,      60, "SMART LOCK");
   create_tile(&g_t_motor, p_scr, 0, 0, (int)FULL_TILE_WIDTH, 40, "COOLING MOTOR");

   ui_reflow((int)g_reed_count);
}

/**
 * \brief  Refresh all tile labels and status dots from the current sensor state.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_update(void)
{
   char     buf[SNPRINTF_BUF] = {0}; /**< Scratch buffer for formatted strings */
   uint32_t now               = 0ul; /**< Current HAL tick in ms               */
   uint8_t  i                 = 0u;  /**< Reed tile loop index                 */

   now = HAL_GetTick();

   /* Recompute online flags from timestamps */
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

   /* Temperature tile */
   (void)snprintf(buf, sizeof(buf), "%uC  %u%%", g_home.temp, g_home.hum);
   lv_label_set_text(g_t_temp.p_value, buf);
   set_status(g_t_temp.p_status, g_dev_online[eDEV_TEMP]);

   /* PIR tile */
   (void)snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_home.pir_count);
   lv_label_set_text(g_t_pir.p_value, buf);
   if (g_home.pir_batt > 0u)
   {
      (void)snprintf(buf, sizeof(buf), "BATT %u%%", g_home.pir_batt);
      lv_label_set_text(g_t_pir.p_sub, buf);
   }
   set_status(g_t_pir.p_status, g_dev_online[eDEV_PIR]);

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

      /* Battery sub-label */
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

/**
 * \brief  Return the current active reed count.
 *
 * \param  void
 *
 * \return uint8_t - Active reed count.
 *
 * \author MichaelLynnCSU
 */
uint8_t ui_get_reed_count(void)
{
   return g_reed_count;
}

/**
 * \brief  Set the active reed sensor count (clamped to MAX_REEDS).
 *
 * \param  count - New reed count.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_set_reed_count(uint8_t count)
{
   if (count > (uint8_t)MAX_REEDS)
   {
      count = (uint8_t)MAX_REEDS;
   }

   g_reed_count = count;
}

/**
 * \brief  Record the current tick as the last-seen time for a named device.
 *
 * \param  dev_id - Device identifier (must be < eDEV_COUNT).
 * \param  tick   - HAL_GetTick() value to record.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_stamp_dev_online(DEVICE_ID_E dev_id, uint32_t tick)
{
   if (dev_id < eDEV_COUNT)
   {
      g_dev_last_seen[dev_id] = tick;
   }
}

/**
 * \brief  Record the current tick as the last-seen time for a reed sensor.
 *
 * \param  slot - Zero-based reed index (must be < MAX_REEDS).
 * \param  tick - HAL_GetTick() value to record.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_stamp_reed_online(uint8_t slot, uint32_t tick)
{
   if (slot < (uint8_t)MAX_REEDS)
   {
      g_reed_last_seen[slot] = tick;
   }
}

/**
 * \brief  Return the current online flag for a named device.
 *
 * \param  dev_id - Device to query (must be < eDEV_COUNT).
 *
 * \return uint8_t - 1 if online, 0 if offline or id out of range.
 *
 * \author MichaelLynnCSU
 */
uint8_t ui_get_dev_online(DEVICE_ID_E dev_id)
{
   if (dev_id >= eDEV_COUNT)
   {
      return 0u;
   }

   return g_dev_online[dev_id];
}

void ui_set_temp(uint8_t val)       { g_home.temp = val; }
void ui_set_hum(uint8_t val)        { g_home.hum  = val; }
void ui_set_pir_count(uint32_t val) { g_home.pir_count = val; }
void ui_set_pir_batt(uint8_t val)   { g_home.pir_batt  = val; }
void ui_set_light(uint8_t val)      { g_home.light = val; }
void ui_set_lock(uint8_t val)       { g_home.lock  = val; }
void ui_set_lock_batt(int8_t val)   { g_home.lock_batt = val; }
void ui_set_motor(uint8_t val)      { g_home.motor = val; }

/**
 * \brief  Set the door state for a reed sensor slot.
 *
 * \param  slot  - Zero-based reed index.
 * \param  state - 0 = closed, 1 = open.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_set_reed_state(uint8_t slot, uint8_t state)
{
   if (slot < (uint8_t)MAX_REEDS)
   {
      g_home.reed_state[slot] = state;
   }
}

/**
 * \brief  Set the battery level for a reed sensor slot.
 *
 * \param  slot - Zero-based reed index.
 * \param  batt - Battery percent (0-100), or -1 if unknown.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_set_reed_batt(uint8_t slot, int8_t batt)
{
   if (slot < (uint8_t)MAX_REEDS)
   {
      g_home.reed_batt[slot] = batt;
   }
}

/**
 * \brief  Set the BLE advertisement age for a reed sensor slot.
 *
 * \param  slot - Zero-based reed index.
 * \param  age  - Age in seconds since last BLE advertisement.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_set_reed_age(uint8_t slot, uint16_t age)
{
   if (slot < (uint8_t)MAX_REEDS)
   {
      g_home.reed_age[slot] = age;
   }
}

void ui_set_motor_batt(int val)
{
   g_home.motor_batt = val;
}
