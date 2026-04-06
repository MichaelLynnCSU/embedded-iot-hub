#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define TCP_PORT             3333
#define MOTOR_LOOP_MS        100
#define STATS_INTERVAL_MS    60000u
#define RECV_TIMEOUT_MS      30000
#define ACCEPT_TIMEOUT_SEC   2
#define ADC_MAX_RAW          4095
#define PWM_DUTY_RES         13
#define PWM_DUTY_MAX         ((1 << PWM_DUTY_RES) - 1)
#define DEFAULT_AWS_LOW      20
#define DEFAULT_AWS_HIGH     35
#define DEFAULT_AVG_TEMP     25
#define LOG_THROTTLE_N       20u

static inline uint32_t motor_temp_to_duty(int t, int low, int high)
{
    if (t < low)  { t = low;  }
    if (t > high) { t = high; }
    if (high <= low) { return 0; }
    return (uint32_t)PWM_DUTY_MAX *
           (uint32_t)(t - low) /
           (uint32_t)(high - low);
}

#endif /* MAIN_H */
