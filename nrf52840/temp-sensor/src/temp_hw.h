#ifndef TEMP_HW_H
#define TEMP_HW_H
#include <stdint.h>
int    temp_hw_init(void);
int    temp_hw_read_raw(int16_t *out_raw);
#endif /* TEMP_HW_H */
