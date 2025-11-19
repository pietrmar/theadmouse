#pragma once

#include <zephyr/device.h>
#include <zephyr/input/input.h>

extern const struct device *const dev_headmouse_input_kbd;
extern const struct device *const dev_headmouse_input_mouse;

static inline int hm_input_report_rel(uint16_t code, int32_t value, bool sync, k_timeout_t timeout)
{
	return input_report_rel(dev_headmouse_input_mouse, code, value, sync, timeout);
}
