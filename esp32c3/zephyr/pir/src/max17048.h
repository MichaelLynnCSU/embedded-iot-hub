#pragma once
int max17048_init(void);
int max17048_read_soc(void);     /* returns 0-100 % */
int max17048_read_mv(void);      /* returns cell voltage in mV */
