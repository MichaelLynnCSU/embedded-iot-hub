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
 *
 * \note    Doorbell liveness (2026-06-09):
 *          MAX_DOORBELL_CAMS added. ui_set_doorbell_slot_age(),
 *          ui_set_doorbell_slot_online(), ui_stamp_doorbell_online() added.
 *
 * \note    Inference camera liveness (2026-06-10):
 *          MAX_CAMS=3 added. ui_stamp_cam_online() added.
 *          ui_set_cam() carries display config only — no liveness authority.
 *          Offline transitions are driven solely by HB_TIMEOUT_MS in
 *          ui_tick(). Do NOT add an online parameter to ui_set_cam().
 *          SYSTEM view shows CAM1..CAM3 rows with online/offline status.
 *
 * \note    Inference-aware doorbell (2026-06-14):
 *          ui_set_doorbell_result() added. Extends ui_set_doorbell() with
 *          person, conf_pct, and asset fields. asset is a 16-char ISO 8601
 *          timestamp token (e.g. 20260614T182513Z) stored in char[20].
 *          press->inference delta is 1-4 s by design; the UI pending window
 *          (g_doorbell_pending + DOORBELL_UI_TIMEOUT_MS) absorbs this gap.
 *          ui_set_doorbell() is retained as a zero-inference shim.
 *
 * \note    Wire age vs online last-seen (2026-06-26):
 *          TWO SEPARATE CONCEPTS — do not confuse them:
 *
 *          WIRE AGE — ui_set_lock_age(), ui_set_light_age(), ui_set_pir_age(),
 *          ui_set_motor_age(), ui_set_temp_age():
 *            The age value carried in the incoming UART frame from the ESP32.
 *            Represents seconds since the ESP32 hub last heard from that
 *            device over BLE/WiFi. Stored in g_home.*_age fields.
 *            Used for display only ("A:Xs" in SYSTEM view).
 *            AGE_UNKNOWN_VAL (0xFFFF) = hub has never seen the device.
 *            Does NOT drive the online/offline dot — that is last-seen stamps.
 *
 *          ONLINE LAST-SEEN STAMP — ui_stamp_dev_online(), ui_stamp_reed_online(),
 *          ui_stamp_pir_online(), ui_stamp_temp_online(), etc.:
 *            A local HAL_GetTick() timestamp written when a fresh wire frame
 *            arrives (age < WIRE_AGE_ONLINE_THRESHOLD_S in parser.c).
 *            Used in ui_update() to compute g_dev_online[] via HB_TIMEOUT_MS.
 *            Drives the green/red dot and all_online flag.
 *            NOT shown directly in the UI.
 ******************************************************************************/

#ifndef INCLUDE_UI_H_
#define INCLUDE_UI_H_

#include "main.h"
#include "lvgl.h"
#include <stdint.h>

/******************************** CONSTANTS ***********************************/

#define MAX_REEDS          6u    /**< Maximum number of reed sensors           */
#define MAX_PIRS           5u    /**< Maximum number of per-slot PIR sensors   */
#define MAX_TEMPS          4u    /**< Maximum number of BLE temp sensors       */
#define MAX_DOORBELL_CAMS  4u    /**< Maximum number of doorbell cameras       */
#define MAX_CAMS           3u    /**< Number of inference cameras              */

#define HB_TIMEOUT_MS          30000ul  /**< ms before a device is considered offline (network reality model)  */
#define DOORBELL_UI_TIMEOUT_MS 10000ul  /**< ms to hold doorbell alert on LCD (human perception window)        */

#define TILE_GAP           4u   /**< Pixel gap between adjacent tiles          */
#define TILE_LEFT_MARGIN   5u   /**< Left edge x-coordinate for left column    */
#define TILE_RIGHT_COL_X 125u   /**< Left edge x-coordinate for right column   */
#define TILE_WIDTH       110u   /**< Standard tile width in pixels              */
#define FULL_TILE_WIDTH  230u   /**< Full-width tile width                      */

#define HDR_HEIGHT        28u   /**< Header bar height                         */
#define NAV_HEIGHT        28u   /**< Bottom nav bar height in pixels            */
#define NAV_ZONE_W        80u   /**< Width of each nav tap zone (240 / 3)      */
#define DISP_VER_RES_H   320u   /**< Display vertical resolution               */
#define DISP_HOR_RES_H   240u   /**< Display horizontal resolution             */

#define CONTENT_TOP      (HDR_HEIGHT + TILE_GAP)   /**< Layout-static: assumes fixed HDR_HEIGHT */
#define CONTENT_BOT      (DISP_VER_RES_H - NAV_HEIGHT)
#define CONTENT_H        (CONTENT_BOT - CONTENT_TOP)

/******************************* ENUMERATIONS *********************************/

typedef enum
{
   eDEV_TEMP  = 0,
   eDEV_PIR,
   eDEV_LIGHT,
   eDEV_LOCK,
   eDEV_MOTOR,
   eDEV_COUNT
} DEVICE_ID_E;

typedef enum
{
   eVIEW_HOME     = 0,
   eVIEW_SECURITY = 1,
   eVIEW_SYSTEM   = 2,
   eVIEW_COUNT
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
void      ui_poll_touch(void);

/* ---- Reed / PIR counts ---- */
uint8_t ui_get_reed_count(void);
void    ui_set_reed_count(uint8_t count);
uint8_t ui_get_pir_count_slots(void);
void    ui_set_pir_count_slots(uint8_t count);

/* ---- Online stamping ----
 * Write a local HAL_GetTick() timestamp when a fresh wire frame is received.
 * These drive g_dev_online[] / g_reed_online[] / etc. via HB_TIMEOUT_MS.
 * They are NOT wire ages and are NOT shown in the UI directly.
 * See "Wire age vs online last-seen" note above.                            */
void ui_stamp_dev_online(DEVICE_ID_E dev_id, uint32_t tick);
void ui_stamp_reed_online(uint8_t slot, uint32_t tick);
void ui_stamp_pir_online(uint8_t slot, uint32_t tick);
void ui_stamp_doorbell_online(uint8_t slot, uint32_t tick);
void ui_stamp_cam_online(uint8_t slot, uint32_t tick);

/* ---- Wire age setters ----
 * Store the age value from the incoming ESP32 wire frame for display.
 * These are shown as "A:Xs" in the SYSTEM view rows.
 * They do NOT affect online/offline status — that is stamp functions above.
 * AGE_UNKNOWN_VAL (0xFFFF) means the hub has never seen the device.
 * See "Wire age vs online last-seen" note above.                            */
void ui_set_lock_age(uint16_t age);
void ui_set_light_age(uint16_t age);
void ui_set_pir_age(uint16_t age);
void ui_set_motor_age(uint16_t age);
void ui_set_temp_age(uint16_t age);

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

/* ---- Temp slot setters ---- */
void    ui_set_temp_count_slots(uint8_t count);
uint8_t ui_get_temp_count_slots(void);
void    ui_set_temp_slot_decidegc(uint8_t slot, int16_t val);
void    ui_set_temp_slot_batt(uint8_t slot, int8_t batt);
void    ui_set_temp_slot_age(uint8_t slot, uint16_t age);
void    ui_stamp_temp_online(uint8_t slot, uint32_t tick);
void    ui_reflow_temp(int n);

/* ---- Doorbell slot setters ---- */
void ui_set_doorbell_slot_age(uint8_t slot, uint16_t age_s);
void ui_set_doorbell_slot_online(uint8_t slot, uint8_t online);

/* ---- Inference camera config ---- */
/* NOTE: ui_set_cam() is config/display metadata only. It carries NO liveness
 * authority. Do NOT add an online parameter — doing so reintroduces
 * dual-authority state. Liveness is driven exclusively by ui_stamp_cam_online()
 * and the HB_TIMEOUT_MS watchdog in ui_tick().                               */
void ui_set_cam(uint8_t slot);
void ui_set_cam_age(uint8_t slot, uint16_t age); /**< wire age_s from CAM frame; AGE_UNKNOWN_VAL until first frame */

/* ---- Reed setters ---- */
void ui_set_reed_state(uint8_t slot, uint8_t state);
void ui_set_reed_batt(uint8_t slot, int8_t batt);
void ui_set_reed_age(uint8_t slot, uint16_t age);

void ui_set_light(uint8_t val);
void ui_set_lock(uint8_t val);
void ui_set_lock_batt(int8_t val);
void ui_set_motor(uint8_t val);
void ui_set_motor_batt(int val);

/* ---- Doorbell event setters ---- */
void ui_set_doorbell(uint8_t pressed, uint8_t device_id);  /**< Legacy shim — calls ui_set_doorbell_result() with zero inference fields */

void ui_set_doorbell_result(uint8_t pressed, uint8_t device_id,
                            uint8_t person, uint8_t conf_pct,
                            const char *p_asset);

uint8_t ui_get_dev_online(DEVICE_ID_E dev_id);

#endif /* INCLUDE_UI_H_ */
