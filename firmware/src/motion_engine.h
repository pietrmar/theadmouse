#pragma once

#include "common.h"

enum mouse_mode {
	MOUSE_MODE_ALT = 0,
	MOUSE_MODE_MOUSE = 1,
	MOUSE_MODE_JOYSTICK_1 = 2,
	MOUSE_MODE_JOYSTICK_2 = 3,
	MOUSE_MODE_JOYSTICK_3 = 4,

	MOUSE_MODE_MAX,
};

static inline bool mouse_mode_valid(enum mouse_mode mode)
{
	return mode >= 0 && mode < MOUSE_MODE_MAX;
}

int motion_engine_init(void);

enum mouse_mode motion_engine_get_mouse_mode(void);
int motion_engine_set_mouse_mode(enum mouse_mode m);

int motion_engine_set_acceleration(float v, enum axis axis);
float motion_engine_get_acceleration(enum axis axis);

int motion_engine_set_deadzone(float v, enum axis axis);
float motion_engine_get_deadzone(enum axis axis);

void motion_engine_reset_absolute_pos(void);
int motion_engine_get_absolute_pos(float *x, float *y);
