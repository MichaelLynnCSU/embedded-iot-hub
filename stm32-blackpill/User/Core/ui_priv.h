/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ui_priv.h
 * \author  MichaelLynnCSU
 * \date    2026-06-14
 *
 * \brief   Private shared types, state, and constants for the UI layer.
 *
 * \details Included only by ui.c, ui_home.c, ui_security.c, ui_system.c.
 *          Never included by parser.c, main.c, or any module outside the
 *          UI layer. The public API surface is ui.h only.
 ******************************************************************************/

#ifndef UI_PRIV_H_
#define UI_PRIV_H_

#include "ui.h"
#include "lvgl.h"
#include <stdint.h>
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
#define PIR_TITLE_BUF      16u
#define SNPRINTF_BUF       48u
#define AGE_UNKNOWN_VAL    0xFFFFu

#define TILE_H_MIN_SEC     36
#define TILE_H_MAX_SEC     72

#define SYS_ROW_COUNT  (4u + MAX_TEMPS + MAX_PIRS + MAX_REEDS + MAX_DOORBELL_CAMS + MAX_CAMS)

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
   uint8_t  doorbell_person;
   uint8_t  doorbell_conf_pct;
   char     doorbell_asset[20];           /* 16-char ISO token + null, 3 bytes margin */
   uint16_t doorbell_slot_age[MAX_DOORBELL_CAMS];
   uint8_t  doorbell_slot_online[MAX_DOORBELL_CAMS];
} HOME_STATE_X;

/************************ EXTERN SHARED STATE *********************************/

/* Defined in ui.c — all ui_*.c modules reference these externs */

extern HOME_STATE_X g_home;

extern uint32_t g_dev_last_seen[eDEV_COUNT];
extern uint8_t  g_dev_online[eDEV_COUNT];

extern uint32_t g_reed_last_seen[MAX_REEDS];
extern uint8_t  g_reed_online[MAX_REEDS];
extern uint8_t  g_reed_count;

extern uint32_t g_pir_last_seen[MAX_PIRS];
extern uint8_t  g_pir_online[MAX_PIRS];
extern uint8_t  g_pir_count_slots;

extern uint32_t g_temp_last_seen[MAX_TEMPS];
extern uint8_t  g_temp_online[MAX_TEMPS];
extern uint8_t  g_temp_count_slots;

extern uint32_t g_doorbell_last_seen[MAX_DOORBELL_CAMS];
extern uint8_t  g_doorbell_online[MAX_DOORBELL_CAMS];

extern uint32_t g_cam_last_seen[MAX_CAMS];
extern uint8_t  g_cam_online[MAX_CAMS];
extern char     g_cam_buf[MAX_CAMS][32];

extern UI_VIEW_E  g_current_view;

extern lv_obj_t  *g_nav_bar;
extern lv_obj_t  *g_nav_lbl[eVIEW_COUNT];

extern lv_style_t g_style_online;
extern lv_style_t g_style_offline;

/* Security view tiles — defined in ui_security.c */
extern TILE_X g_t_pir_slot[MAX_PIRS];
extern TILE_X g_t_reed[MAX_REEDS];
extern TILE_X g_t_lock;

/* System view list — defined in ui_system.c */
extern lv_obj_t *g_sys_list;
extern lv_obj_t *g_sys_rows[SYS_ROW_COUNT];

/* Home view labels — defined in ui_home.c */
extern lv_obj_t *g_home_status_lbl;
extern lv_obj_t *g_home_temp_lbl;
extern lv_obj_t *g_home_net_lbl;

/******************* INTERNAL FUNCTION PROTOTYPES *****************************/

/* ui.c */
void set_status(lv_obj_t *p_box, uint8_t online);
void create_tile(TILE_X *p_t, lv_obj_t *p_parent,
                 int x, int y, int w, int h,
                 const char *p_title);
void nav_highlight(UI_VIEW_E view);
void apply_view(UI_VIEW_E view);

/* ui_home.c */
void create_home_labels(lv_obj_t *p_scr);
void ui_home_update(uint8_t all_online);

/* ui_security.c */
void create_security_tiles(lv_obj_t *p_scr);
void reflow_security(void);
void ui_security_update(void);

/* ui_system.c */
void create_sys_list(lv_obj_t *p_scr);
void system_list_refresh(void);

#endif /* UI_PRIV_H_ */
