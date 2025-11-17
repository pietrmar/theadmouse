#pragma once

#include <stdint.h>

#define SLOT_MANAGER_BASE_PATH		"/lfs"
#define SLOT_MANAGER_SCHEMA_VERSION	1
#define SLOT_MANAGER_MAX_SLOTS		8

#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 1, 2)))
#endif
typedef int (*slot_printf_fn)(const char *fmt, ...);

struct slot_settings
{
	uint32_t color;
};

int slot_manager_init(void);

// TODO: Consider some nicer "iterator" API or so
int slot_manager_list_all_slots(slot_printf_fn printf_fn);
int slot_manager_dump_all_slots(slot_printf_fn printf_fn);

int slot_manager_set_color(uint32_t col);

int slot_manager_save_current_slot_by_name(const char *name);
int slot_manager_load_slot_by_index(int idx);
int slot_manager_load_slot_by_name(const char *name);

int slot_manager_load_next_slot(void);
