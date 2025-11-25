#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble_hid_service.h"
#include "hm_hid_evt.h"

LOG_MODULE_REGISTER(headmouse_input, CONFIG_HEADMOUSE_INPUT_LOG_LEVEL);

int hm_input_report_key(uint8_t hid_keycode, bool pressed, k_timeout_t timeout)
{
	struct hm_hid_evt evt = {
		.type = HM_HID_EVT_KEY,
		.key = {
			.hid_keycode = hid_keycode,
			.pressed = pressed,
		},
	};

	return ble_hid_queue_evt(&evt, timeout);
}

int hm_input_report_mouse_move(int16_t dx, int16_t dy, k_timeout_t timeout)
{
	int ret = 0;

	// TODO: Consider moving this into the BLE HID thread to potentially allow for
	// better coalescing and such.
	while (dx != 0 || dy != 0) {
		int8_t step_dx;
		int8_t step_dy;

		if (dx > 127)
			step_dx = 127;
		else if (dx < -127)
			step_dx = -127;
		else
			step_dx = dx;

		if (dy > 127)
			step_dy = 127;
		else if (dy < -127)
			step_dy = -127;
		else
			step_dy = dy;

		struct hm_hid_evt evt = {
			.type = HM_HID_EVT_MOUSE_MOVE,
			.mouse_move = {
				.dx = step_dx,
				.dy = step_dy,
			},
		};

		ret = ble_hid_queue_evt(&evt, timeout);
		if (ret < 0)
			return ret;

		dx -= step_dx;
		dy -= step_dy;
	}

	return ret;
}

int hm_input_report_mouse_wheel(int8_t steps, k_timeout_t timeout)
{
	struct hm_hid_evt evt = {
		.type = HM_HID_EVT_MOUSE_WHEEL,
		.mouse_wheel = {
			.steps = steps,
		},
	};

	return ble_hid_queue_evt(&evt, timeout);
}

int hm_input_report_mouse_btn(enum hm_hid_mouse_btn btn, bool pressed, k_timeout_t timeout)
{
	struct hm_hid_evt evt = {
		.type = HM_HID_EVT_MOUSE_BTN,
		.mouse_btn = {
			.btn = btn,
			.pressed = pressed,
		},
	};

	return ble_hid_queue_evt(&evt, timeout);
}
