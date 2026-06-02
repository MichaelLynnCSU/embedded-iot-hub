/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ui.h
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   LVGL UI layer public interface for the Smart Home Dashboard.
 *
 * \details Exposes tile creation, layout reflow, sensor-state setters, and
 *          device-online stamping. The parser and main modules use these
 *          accessor functions instead of touching HomeState directly.
 *
 * \note    Multi-view (2026-06-01):
 *          UI_VIEW_E added. Three views: HOME, SECURITY, SYSTEM.
 *          ui_set_view() / ui_get_view() / ui_poll_touch() exposed.
 *          CONTENT_TOP / CONTENT_BOT / CONTENT_H macros encode the
 *          pixel budget between header and nav bar.
 ******************************************************************************/

#ifndef INCLUDE_UI_H_
#define INCLUDE_UI_H_

#include "main.h"
#include "lvgl.h"
#include <stdint.h>

/******************************** CONSTANTS ***********************************/

#define MAX_REEDS          6u    /**< Maximum number of reed sensors supported */
#define MAX_PIRS           5u    /**< Maximum number of per-slot PIR sensors   */
#define HB_TIMEOUT_MS  30000ul  /**< ms before a device is considered offline  */
#define TILE_GAP           4u   /**< Pixel gap between adjacent tiles          */
#define TILE_LEFT_MARGIN   5u   /**< Left edge x-coordinate for left column    */
#define TILE_RIGHT_COL_X 125u   /**< Left edge x-coordinate for right column   */
#define TILE_WIDTH       110u   /**< Standard tile width in pixels              */
#define FULL_TILE_WIDTH  230u   /**< Full-width tile width                      */

#define HDR_HEIGHT        28u   /**< Header bar height — shared with ui.c      */
#define NAV_HEIGHT        28u   /**< Bottom nav bar height in pixels            */
#define NAV_ZONE_W        80u   /**< Width of each nav tap zone (240 / 3)      */
#define DISP_VER_RES_H   320u   /**< Display vertical resolution               */
#define DISP_HOR_RES_H   240u   /**< Display horizontal resolution             */

#define CONTENT_TOP      (HDR_HEIGHT + TILE_GAP)          /**< First usable y  */
#define CONTENT_BOT      (DISP_VER_RES_H - NAV_HEIGHT)   /**< Last usable y   */
#define CONTENT_H        (CONTENT_BOT - CONTENT_TOP)     /**< Usable px height */

/******************************* ENUMERATIONS *********************************/

/**
 * \brief Device ID enumeration used to index online-tracking arrays.
 */
typedef enum
{
   eDEV_TEMP  = 0, /*!< Temperature / humidity sensor */
   eDEV_PIR,       /*!< PIR motion sensor (aggregate) */
   eDEV_LIGHT,     /*!< Smart light                   */
   eDEV_LOCK,      /*!< Smart lock                    */
   eDEV_MOTOR,     /*!< Cooling/heating motor          */
   eDEV_COUNT      /*!< Must be last — array size      */
} DEVICE_ID_E;

/**
 * \brief Active view / page enum.
 */
typedef enum
{
   eVIEW_HOME     = 0,  /*!< Summary — "is the house okay?"    */
   eVIEW_SECURITY = 1,  /*!< PIR slots + reeds + lock          */
   eVIEW_SYSTEM   = 2,  /*!< Per-device online / battery / age */
   eVIEW_COUNT          /*!< Must be last                       */
} UI_VIEW_E;

/*************************** FUNCTION PROTOTYPES ******************************/

void ui_create(void);
void ui_update(void);
void ui_tick(void);
void ui_reflow(int n);
void ui_reflow_pir(int n);

/* ---- View navigation ---- */
void      ui_set_view(UI_VIEW_E view);
UI_VIEW_E ui_get_view(void);
void      ui_poll_touch(void);   /**< Call once per main loop iteration       */

/* ---- Reed / PIR counts ---- */
uint8_t ui_get_reed_count(void);
void    ui_set_reed_count(uint8_t count);
uint8_t ui_get_pir_count_slots(void);
void    ui_set_pir_count_slots(uint8_t count);

/* ---- Online stamping ---- */
void ui_stamp_dev_online(DEVICE_ID_E dev_id, uint32_t tick);
void ui_stamp_reed_online(uint8_t slot, uint32_t tick);
void ui_stamp_pir_online(uint8_t slot, uint32_t tick);

/* ---- Sensor state setters ---- */
void ui_set_temp(uint8_t val);
void ui_set_hum(uint8_t val);
void ui_set_pir_count(uint32_t val);
void ui_set_pir_batt(uint8_t val);
void ui_set_pir_occupied(uint8_t val);

void ui_set_pir_slot_count(uint8_t slot, uint32_t val);
void ui_set_pir_slot_batt(uint8_t slot, int8_t batt);
void ui_set_pir_slot_age(uint8_t slot, uint16_t age);
void ui_set_pir_slot_occupied(uint8_t slot, uint8_t val);

void ui_set_reed_state(uint8_t slot, uint8_t state);
void ui_set_reed_batt(uint8_t slot, int8_t batt);
void ui_set_reed_age(uint8_t slot, uint16_t age);

void ui_set_light(uint8_t val);
void ui_set_lock(uint8_t val);
void ui_set_lock_batt(int8_t val);
void ui_set_motor(uint8_t val);
void ui_set_motor_batt(int val);

uint8_t ui_get_dev_online(DEVICE_ID_E dev_id);

#endif /* INCLUDE_UI_H_ */
