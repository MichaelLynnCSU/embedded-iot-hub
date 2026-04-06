#ifndef BLUEPILL_LOGIC_H
#define BLUEPILL_LOGIC_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Timing constants ---- */
#define DEFAULT_TASK_DELAY   5000u
#define SENSOR_TASK_DELAY    2000u
#define SENSOR_WARMUP_MS     2000u
#define MUTEX_TIMEOUT_MS     100u
#define UART_TIMEOUT_MS      1000u

/* ---- Sensor constants ---- */
#define SENSOR_COUNT         3u
#define JSON_BUF_SIZE        64u
#define MSG_BUF_SIZE         80u

/* ---- WDT constants ---- */
#define IWDG_RELOAD          1875u   /* F103: LSI ~40kHz / prescaler 64 ~ 3s */
#define TRINITY_MSG_LEN      80u

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

/* ---- RCC_CSR reset reason bit masks (F103) ---- */
#define RCC_CSR_PORRSTF      (1u << 27)  /* power-on reset */
#define RCC_CSR_IWDGRSTF     (1u << 29)  /* IWDG reset */
#define RCC_CSR_WWDGRSTF     (1u << 30)  /* WWDG reset */
#define RCC_CSR_PINRSTF      (1u << 26)  /* pin reset */
#define RCC_CSR_SFTRSTF      (1u << 28)  /* software reset */

/* ---- Pure logic: classify_reset_cause ---- */
static inline TRINITY_ERROR_E bluepill_classify_reset(uint32_t reset_reason)
{
    if      (0u != (reset_reason & RCC_CSR_PORRSTF))
    { return eTRINITY_ERR_BROWNOUT; }
    else if ((0u != (reset_reason & RCC_CSR_IWDGRSTF)) ||
             (0u != (reset_reason & RCC_CSR_WWDGRSTF)))
    { return eTRINITY_ERR_WATCHDOG; }
    else if ((0u != (reset_reason & RCC_CSR_PINRSTF)) ||
             (0u != (reset_reason & RCC_CSR_SFTRSTF)))
    { return eTRINITY_ERR_NONE; }

    return eTRINITY_ERR_UNKNOWN;
}

/* ---- Pure logic: average temperature ---- */
static inline int bluepill_avg_temp(const uint8_t *temps,
                                     const uint8_t *valid,
                                     int count)
{
    int sum         = 0;
    int valid_count = 0;
    int i           = 0;

    for (i = 0; i < count; i++)
    {
        if (valid[i])
        {
            sum += (int)temps[i];
            valid_count++;
        }
    }

    if (0 == valid_count) { return 0; }
    return sum / valid_count;
}

/* ---- Pure logic: fault type name ---- */
static inline const char *bluepill_fault_name(uint32_t fault_type)
{
    switch (fault_type)
    {
        case 1u: return "HARDFAULT";
        case 2u: return "MEMMANAGE";
        case 3u: return "BUSFAULT";
        case 4u: return "USAGEFAULT";
        default: return "FAULT";
    }
}

#endif /* BLUEPILL_LOGIC_H */
