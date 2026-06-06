#ifndef STUB_SETTINGS_H
#define STUB_SETTINGS_H

/*
 * Stub: zephyr/settings/settings.h
 * ble_gatt.c calls settings_load() after bt_enable().
 * Neither is under test -- stub to no-ops so ble_gatt.c compiles cleanly.
 */

#include <stdint.h>
#include <stddef.h>

typedef ssize_t (*settings_read_cb)(void *cb_arg, void *data, size_t len);

static inline int settings_load(void) { return 0; }
static inline int settings_save_one(const char *name,
                                     const void *value, size_t val_len)
{ (void)name;(void)value;(void)val_len; return 0; }
static inline int settings_subsys_init(void) { return 0; }

#define SETTINGS_STATIC_HANDLER_DEFINE(name, pfx, get, set, commit, export) \
    /* settings handler stub */

#endif /* STUB_SETTINGS_H */
