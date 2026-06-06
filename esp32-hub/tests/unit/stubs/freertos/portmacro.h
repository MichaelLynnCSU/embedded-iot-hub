#ifndef STUB_PORTMACRO_H
#define STUB_PORTMACRO_H
#include <stdint.h>
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define taskENTER_CRITICAL(m)   (void)(m)
#define taskEXIT_CRITICAL(m)    (void)(m)
#define pdTRUE  1
#define pdFALSE 0
#endif
