#include <stdbool.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "keymaps/KeyboardLayout.h"

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

struct kbd_layout_entry {
	const char *locale;
	const uint8_t *layout;
};

static const struct kbd_layout_entry kbd_layout_table[] = {
	{ .locale = "da_DK", .layout = KeyboardLayout_da_DK },
	{ .locale = "de_DE", .layout = KeyboardLayout_de_DE },
	{ .locale = "en_US", .layout = KeyboardLayout_en_US },
	{ .locale = "es_ES", .layout = KeyboardLayout_es_ES },
	{ .locale = "fr_FR", .layout = KeyboardLayout_fr_FR },
	{ .locale = "hu_HU", .layout = KeyboardLayout_hu_HU },
	{ .locale = "it_IT", .layout = KeyboardLayout_it_IT },
	{ .locale = "pt_BR", .layout = KeyboardLayout_pt_BR },
	{ .locale = "pt_PT", .layout = KeyboardLayout_pt_PT },
	{ .locale = "sv_SE", .layout = KeyboardLayout_sv_SE },
};

// The default is `en_US`
static const struct kbd_layout_entry *current_kbd_layout = &kbd_layout_table[2];
static struct k_spinlock current_kbd_layout_lock;

const char *hm_input_get_current_kbd_layout_locale(void)
{
	k_spinlock_key_t key = k_spin_lock(&current_kbd_layout_lock);
	const char *locale = current_kbd_layout->locale;
	k_spin_unlock(&current_kbd_layout_lock, key);

	return locale;
}

int hm_input_set_kbd_layout_locale(const char *locale)
{
	for (size_t i = 0; i < ARRAY_SIZE(kbd_layout_table); i++) {
		if (strcmp(locale, kbd_layout_table[i].locale) == 0) {
			k_spinlock_key_t key = k_spin_lock(&current_kbd_layout_lock);
			current_kbd_layout = &kbd_layout_table[i];
			k_spin_unlock(&current_kbd_layout_lock, key);
			return 0;
		}
	}

	return -ENOENT;
}

struct hid_keystroke {
	uint8_t code;
	bool shift;
	bool altgr;
};

struct hid_keystroke ascii_to_hid_keystroke(unsigned char c)
{
	struct hid_keystroke keystroke = { .code = 0, .shift = false, .altgr = false };

	if (c >= 128)
		return keystroke;

	uint8_t k = current_kbd_layout->layout[c];

	if ((k & ALT_GR) == ALT_GR) {
		keystroke.altgr = true;
		k &= 0x3f;
	} else if ((k & SHIFT) == SHIFT) {
		keystroke.shift = true;
		k &= 0x7f;
	}

	if (k == ISO_REPLACEMENT) {
		k = ISO_KEY;
	}

	keystroke.code = k;

	return keystroke;
}

int hm_input_write_string(const char *s)
{
	bool shift_active = false;
	bool altgr_active = false;

	// TODO: Error handling on errors from `hm_input_report_key()`
	while (*s) {
		struct hid_keystroke keystroke = ascii_to_hid_keystroke(*s);

		if (keystroke.code == 0)
			continue;

		if (keystroke.shift != shift_active) {
			shift_active = keystroke.shift;
			hm_input_report_key(HID_KEY_MOD_LSHIFT, shift_active, K_NO_WAIT);
			k_sleep(K_MSEC(10));
		}

		if (keystroke.altgr != altgr_active) {
			altgr_active = keystroke.altgr;
			hm_input_report_key(HID_KEY_MOD_RALT, altgr_active, K_NO_WAIT);
			k_sleep(K_MSEC(10));
		}

		hm_input_report_key(keystroke.code, true, K_NO_WAIT);
		k_sleep(K_MSEC(10));

		hm_input_report_key(keystroke.code, false, K_NO_WAIT);
		k_sleep(K_MSEC(10));

		s++;

	}

	if (shift_active) {
		hm_input_report_key(HID_KEY_MOD_LSHIFT, false, K_NO_WAIT);
		k_sleep(K_MSEC(10));
	}

	if (altgr_active) {
		hm_input_report_key(HID_KEY_MOD_RALT, false, K_NO_WAIT);
		k_sleep(K_MSEC(10));
	}

	return 0;
}
