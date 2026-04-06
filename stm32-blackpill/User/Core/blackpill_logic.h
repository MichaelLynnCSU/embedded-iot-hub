#ifndef BLACKPILL_LOGIC_H
#define BLACKPILL_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

/* ---- Parser constants ---- */
#define UART_LINE_LEN        128u
#define BLE_AGE_THRESHOLD_S  150u
#define STATE_FIELD_COUNT      8u
#define MAX_REEDS              6u
#define DR_DIGIT_OFFSET        2u

/* ---- Main loop constants ---- */
#define HB_CHECK_MS          1000ul
#define UART_QUEUE_DEPTH        8u
#define MAIN_LOOP_DELAY_MS      5u

/* ---- Trinity constants ---- */
#define STACK_CANARY         0xDEADBEEFul
#define STACK_GUARD_WORDS       4u
#define IWDG_RELOAD          3000u
#define TRINITY_MSG_LEN        80u

/* ---- TRINITY_ERROR_E ---- */
typedef enum
{
   eTRINITY_ERR_NONE        = 0x00u,
   eTRINITY_ERR_HARDFAULT   = 0x01u,
   eTRINITY_ERR_STACK       = 0x02u,
   eTRINITY_ERR_HEAP        = 0x03u,
   eTRINITY_ERR_ASSERT      = 0x04u,
   eTRINITY_ERR_BROWNOUT    = 0x05u,
   eTRINITY_ERR_WATCHDOG    = 0x06u,
   eTRINITY_ERR_PANIC       = 0x07u,
   eTRINITY_ERR_UNKNOWN     = 0xFFu,
} TRINITY_ERROR_E;

/* ---- Pure logic: parse_int ---- */
static inline int logic_parse_int(const char *p_str, int *p_result)
{
   char *p_end = NULL;
   long  val   = 0;

   if ((NULL == p_str) || (NULL == p_result)) { return -1; }

   errno = 0;
   val   = strtol(p_str, &p_end, 10);

   if ((errno != 0) || (p_end == p_str)) { return -1; }

   *p_result = (int)val;
   return 0;
}

/* ---- Pure logic: DR slot routing ---- */
static inline bool logic_is_dr_id(const char *p_id)
{
   if (NULL == p_id) { return false; }
   return (('D' == p_id[0]) && ('R' == p_id[1]) &&
           (p_id[DR_DIGIT_OFFSET] >= '1') &&
           (p_id[DR_DIGIT_OFFSET] <= '6'));
}

static inline int logic_dr_slot(const char *p_id)
{
   return (int)(p_id[DR_DIGIT_OFFSET] - '1');
}

/* ---- Pure logic: BLE age online check ---- */
static inline bool logic_ble_is_online(int age_s)
{
   return age_s < (int)BLE_AGE_THRESHOLD_S;
}

/* ---- Pure logic: reed count clamp ---- */
static inline int logic_clamp_reed_count(int n)
{
   if (n > (int)MAX_REEDS) { return (int)MAX_REEDS; }
   if (n < 0)              { return 0; }
   return n;
}

/* ---- Pure logic: stack canary check ---- */
static inline bool logic_canary_intact(const uint32_t *p_base, uint32_t words)
{
   uint32_t i = 0;
   if (NULL == p_base) { return true; } /* no canary — not a failure */
   for (i = 0; i < words; i++)
   {
      if (STACK_CANARY != p_base[i]) { return false; }
   }
   return true;
}

/* ---- Pure logic: STATE field count ---- */
static inline uint8_t logic_state_field_count(void)
{
   return STATE_FIELD_COUNT;
}

#endif /* BLACKPILL_LOGIC_H */
