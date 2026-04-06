#ifndef SERVER_LOGIC_H
#define SERVER_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ---- Constants ---- */
#define MAX_ROOMS        10
#define MAX_REEDS        6
#define BUFFER_SIZE      4096
#define READ_BUF_SIZE    256
#define PIPE_RETRY_SEC   3
#define AGE_MAX          0xFFFEu
#define AGE_UNKNOWN      0xFFFFu
#define REED_NAME_LEN    16
#define ROOM_NAME_LEN    32
#define ROOM_STATE_LEN   16
#define ROOM_LOC_LEN     32

/* ---- Pure logic: age clamping ---- */
static inline uint16_t logic_clamp_age(int v)
{
    if (0 > v)           { return AGE_UNKNOWN; }
    if (v > (int)AGE_MAX) { return (uint16_t)AGE_MAX; }
    return (uint16_t)v;
}

/* ---- Pure logic: reed slot ID to index ---- */
static inline int logic_reed_id_to_slot(int id)
{
    return id - 1;
}

static inline bool logic_reed_slot_valid(int slot)
{
    return (slot >= 0) && (slot < MAX_REEDS);
}

/* ---- Pure logic: extract_json brace counting ---- */
static inline char *logic_extract_json(char *p_buf)
{
    char *p_start = NULL;
    char *p       = NULL;
    int   depth   = 0;

    p_start = strchr(p_buf, '{');
    if (NULL == p_start) { return NULL; }

    for (p = p_start; '\0' != *p; p++)
    {
        if      ('{' == *p) { depth++; }
        else if ('}' == *p)
        {
            depth--;
            if (0 == depth) { *(p + 1) = '\0'; return p_start; }
        }
    }
    return NULL;
}

/* ---- Pure logic: buffer overflow check ---- */
static inline bool logic_buf_has_room(int pos, int n, int size)
{
    return (pos + n) < size;
}

#endif /* SERVER_LOGIC_H */
