#pragma once

#define SLOT_MANAGER_BASE_PATH		"/lfs"
#define SLOT_MANAGER_SCHEMA_VERSION	1
#define SLOT_MANAGER_MAX_SLOTS		8

#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 1, 2)))
#endif
typedef int (*slot_printf_fn)(const char *fmt, ...);

struct slot_settings
{
	uint32_t led_color;
};

int slot_manager_init(void);
int slot_manager_dump_all_slots(slot_printf_fn printf_fn);
