#ifndef STUB_NRF_H
#define STUB_NRF_H
#include <stdint.h>
typedef struct { uint32_t RESETREAS; } NRF_POWER_Type;
static NRF_POWER_Type _stub_nrf_power = {0};
#define NRF_POWER (&_stub_nrf_power)
static inline void NVIC_SystemReset(void) {}
#endif /* STUB_NRF_H */
