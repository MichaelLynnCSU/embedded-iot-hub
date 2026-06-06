#ifndef STUB_LWIP_SOCKETS_H
#define STUB_LWIP_SOCKETS_H
/* Stub: lwip/sockets.h -- motor_sm.h uses sockaddr_in for ping */
#include <stdint.h>
struct in_addr { uint32_t s_addr; };
struct sockaddr_in {
    uint16_t       sin_family;
    uint16_t       sin_port;
    struct in_addr sin_addr;
};
#endif
