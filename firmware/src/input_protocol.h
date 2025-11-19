#pragma once

enum input_mode {
	INPUT_MODE_ALT = 0,
	INPUT_MODE_MOUSE = 1,
	INPUT_MODE_JOYSTICK_1 = 2,
	INPUT_MODE_JOYSTICK_2 = 3,
	INPUT_MODE_JOYSTICK_3 = 4,

	INPUT_MODE_MAX,
};

static inline bool input_mode_valid(enum input_mode mode)
{
	return mode >= 0 && mode < INPUT_MODE_MAX;
}
