#pragma once

#include "common.h"

int motion_engine_init(void);

int motion_engine_set_hid_axis_sensitivity(float v, enum axis axis);
float motion_engine_get_hid_axis_sensitivity(enum axis axis);

void motion_engine_reset_absolute_hid_pos(void);
int motion_engine_get_absolute_hid_pos(float *x, float *y);
