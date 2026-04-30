#pragma once
#include <stdint.h>

/* 4×AA discharge range in mV.
 * VBAT_FULL_MV = 6400: four fresh AA cells at ~1.6V each.
 * VBAT_DEAD_MV = 4400: cutoff below which cells are considered dead.
 * Usable window: 2000mV. */
#define VBAT_FULL_MV      6400
#define VBAT_LOW_MV       5200
#define VBAT_CRITICAL_MV  4800
#define VBAT_DEAD_MV      4400
#define VBAT_RANGE_MV     (VBAT_FULL_MV - VBAT_DEAD_MV)

/* Voltage divider ratio: Vbat → 100k → 100k → AIN0
 *                                              └ → 100k → GND
 * R1 = 200k (two 100k in series), R2 = 100k
 * Ratio = R2 / (R1 + R2) = 100k / 300k = 1/3
 * Reconstruct: adc_pin_mV × DIVIDER_RATIO_DEN = Vbat_mV */
#define DIVIDER_RATIO_NUM  1
#define DIVIDER_RATIO_DEN  3

static inline uint8_t mv_to_soc(int mv)
{
    if (mv >= VBAT_FULL_MV) { return 100u; }
    if (mv <= VBAT_DEAD_MV) { return 0u;   }
    return (uint8_t)((mv - VBAT_DEAD_MV) * 100 / VBAT_RANGE_MV);
}

int  battery_init(void);
int  battery_read_mv(void);
void battery_print_status(void);
