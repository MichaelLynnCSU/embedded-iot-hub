#include <zephyr/kernel.h>

__attribute__((used, retain, section(".rodata"))) 
const char trinity_build_mode[] = "TRINITY_BUILD_MODE=" CONFIG_TRINITY_MODE_STR;

__attribute__((used, retain, section(".rodata"))) 
const char trinity_build_date[] = "TRINITY_BUILD_DATE=" __DATE__ " " __TIME__;
