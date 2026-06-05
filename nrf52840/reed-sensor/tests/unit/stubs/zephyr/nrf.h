#ifndef STUB_NRF_H
#define STUB_NRF_H

/*
 * Stub: nrf.h
 * main.c reads and clears NRF_POWER->RESETREAS at the top of main().
 * trinity_fault.c reads SCB registers.
 * Neither main() nor the fault handler is compiled into the unit test
 * build, but the header is included transitively. Provide the minimal
 * struct so it compiles cleanly.
 */

#include <stdint.h>

typedef struct {
    uint32_t RESETREAS;
} NRF_POWER_Type;

static NRF_POWER_Type _stub_nrf_power = {0};
#define NRF_POWER (&_stub_nrf_power)

static inline void NVIC_SystemReset(void) {}

#endif /* STUB_NRF_H */
