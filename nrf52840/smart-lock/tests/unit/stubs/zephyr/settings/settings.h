#ifndef STUB_SETTINGS_H
#define STUB_SETTINGS_H

/*
 * Stub: zephyr/settings/settings.h
 * ble_gatt.c calls settings_load() and settings_save_one().
 * Neither is under test -- stub to no-ops so ble_gatt.c compiles.
 */

#include <stdint.h>
#include <stddef.h>

typedef ssize_t (*settings_read_cb)(void *cb_arg, void *data, size_t len);

static inline int settings_load(void) { return 0; }
static inline int settings_save_one(const char *name,
                                     const void *value, size_t val_len)
{ (void)name;(void)value;(void)val_len; return 0; }
static inline int settings_subsys_init(void) { return 0; }

/* SETTINGS_STATIC_HANDLER_DEFINE -- swallow the handler registration */
#define SETTINGS_STATIC_HANDLER_DEFINE(name, pfx, get, set, commit, export) \
    /* settings handler stub: name##_settings_handler */

#endif /* STUB_SETTINGS_H */
