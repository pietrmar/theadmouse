#pragma once

#include "common.h"

int motion_engine_init(void);

int motion_engine_set_acceleration(float v, enum axis axis);
float motion_engine_get_acceleration(enum axis axis);

void motion_engine_reset_absolute_pos(void);
int motion_engine_get_absolute_pos(float *x, float *y);
