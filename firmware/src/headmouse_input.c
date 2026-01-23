#include <stdbool.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "keymaps/KeyboardLayout.h"

#include "ble_hid_service.h"
#include "hm_hid_evt.h"

LOG_MODULE_REGISTER(headmouse_input, CONFIG_HEADMOUSE_INPUT_LOG_LEVEL);

struct keyname_entry {
	const char *name;
	// Internal code, has the following zones:
	// - 0x00: No character
	// - 0x00 - 0x0F: ASCII (mainly for BS, TAB and LF)
	// - 0x10 - 0x17: Modifier keys 0xE0 - 0xE7
	// - 0x20 - 0x7F: Printable ASCII range
	// - 0x80 - 0xFF: Directly encoded HID codes
	uint8_t icode;
};

#define HID_TO_ICODE(x) (0x80 + (x))
#define MOD_TO_ICODE(x) ((x) - 0xE0 + 0x10)

static struct keyname_entry keyname_to_icode_map[] = {
	{ "A", 'a' }, { "B", 'b' }, { "C", 'c' }, { "D", 'd' }, { "E", 'e' },
	{ "F", 'f' }, { "G", 'g' }, { "H", 'h' }, { "I", 'i' }, { "J", 'j' },
	{ "K", 'k' }, { "L", 'l' }, { "M", 'm' }, { "N", 'n' }, { "O", 'o' },
	{ "P", 'p' }, { "Q", 'q' }, { "R", 'r' }, { "S", 's' }, { "T", 't' },
	{ "U", 'u' }, { "V", 'v' }, { "W", 'w' }, { "X", 'x' }, { "Y", 'y' },
	{ "Z", 'z' },

	{ "1", '1' }, { "2", '2' }, { "3", '3' }, { "4", '4' }, { "5", '5' },
	{ "6", '6' }, { "7", '7' }, { "8", '8' }, { "9", '9' }, { "0", '0' },

	{ "ENTER", '\n' }, { "BACKSPACE", '\b' }, { "TAB", '\t' }, { "SPACE", ' ' },

	{ "SLASH", '/' }, { "BACKSLASH", '\\' }, { "LEFT_BRACE", '(' },
	{ "RIGHT_BRACE", ')' }, { "QUOTE", '"' }, { "TILDE", '~' },
	{ "MINUS", '-' }, { "SEMICOLON", ';' }, { "EQUAL", '=' },
	{ "COMMA", ',' }, { "PERIOD", '.' },

	{ "ESC" , HID_TO_ICODE(HID_KEY_ESC) },

	{ "F1", HID_TO_ICODE(HID_KEY_F1) }, { "F2", HID_TO_ICODE(HID_KEY_F2) },
	{ "F3", HID_TO_ICODE(HID_KEY_F3) }, { "F4", HID_TO_ICODE(HID_KEY_F4) },
	{ "F5", HID_TO_ICODE(HID_KEY_F5) }, { "F6", HID_TO_ICODE(HID_KEY_F6) },
	{ "F7", HID_TO_ICODE(HID_KEY_F7) }, { "F8", HID_TO_ICODE(HID_KEY_F8) },
	{ "F9", HID_TO_ICODE(HID_KEY_F9) }, { "F10", HID_TO_ICODE(HID_KEY_F10) },
	{ "F11", HID_TO_ICODE(HID_KEY_F11) }, { "F12", HID_TO_ICODE(HID_KEY_F12) },

	{ "RIGHT", HID_TO_ICODE(HID_KEY_RIGHT) }, { "LEFT", HID_TO_ICODE(HID_KEY_LEFT) },
	{ "DOWN", HID_TO_ICODE(HID_KEY_DOWN) }, { "UP", HID_TO_ICODE(HID_KEY_UP) },


	{ "PAGE_UP", HID_TO_ICODE(HID_KEY_PAGEUP) }, { "PAGE_DOWN", HID_TO_ICODE(HID_KEY_PAGEDOWN) },
	{ "DELETE", HID_TO_ICODE(HID_KEY_DELETE) }, { "INSERT", HID_TO_ICODE(HID_KEY_INSERT) },
	{ "HOME", HID_TO_ICODE(HID_KEY_HOME) }, { "END", HID_TO_ICODE(HID_KEY_END) },
	{ "NUM_LOCK", HID_TO_ICODE(HID_KEY_NUMLOCK) }, { "SCROLL_LOCK", HID_TO_ICODE(HID_KEY_SCROLLLOCK) },

	{ "CAPS_LOCK", HID_TO_ICODE(HID_KEY_CAPSLOCK) }, { "PAUSE", HID_TO_ICODE(HID_KEY_PAUSE) },
	{ "MENU", HID_TO_ICODE(0x65) }, // HID_KEY_APPLICATION

	{ "SHIFT", MOD_TO_ICODE(HID_KEY_MOD_LSHIFT) },
	{ "CTRL", MOD_TO_ICODE(HID_KEY_MOD_LCTRL) },
	{ "ALT", MOD_TO_ICODE(HID_KEY_MOD_LALT) },
	{ "RIGHT_ALT", MOD_TO_ICODE(HID_KEY_MOD_RALT) },
	{ "GUI", MOD_TO_ICODE(HID_KEY_MOD_LGUI) },
	{ "RIGHT_GUI",  MOD_TO_ICODE(HID_KEY_MOD_LGUI) },
};

static struct keyname_entry keypad_to_icode_map[] = {
	{ "1", HID_TO_ICODE(HID_KEY_KP_1) }, { "2", HID_TO_ICODE(HID_KEY_KP_2) }, { "3", HID_TO_ICODE(HID_KEY_KP_3) },
	{ "4", HID_TO_ICODE(HID_KEY_KP_4) }, { "5", HID_TO_ICODE(HID_KEY_KP_5) }, { "6", HID_TO_ICODE(HID_KEY_KP_6) },
	{ "7", HID_TO_ICODE(HID_KEY_KP_7) }, { "8", HID_TO_ICODE(HID_KEY_KP_8) }, { "9", HID_TO_ICODE(HID_KEY_KP_9) },
	{ "0", HID_TO_ICODE(HID_KEY_KP_0) },

	{ "SLASH", HID_TO_ICODE(HID_KEY_KPSLASH) },
	{ "MINUS", HID_TO_ICODE(HID_KEY_KPMINUS) },
	{ "PLUS", HID_TO_ICODE(HID_KEY_KPPLUS) },
	{ "ENTER", HID_TO_ICODE(HID_KEY_KPENTER) },
	{ "PERIOD", HID_TO_ICODE(0x63) }, // HID_KEY_KPPERIOD
	{ "ASTERIX", HID_TO_ICODE(HID_KEY_KPASTERISK) },	// This is a typo or an easter-egg :) in the FABI firmware, keep it for backwards compatibility
};

int hm_input_icode_to_keyname(char *buf, size_t len, uint8_t icode)
{
	const char *prefix = NULL;
	const char *keyname = NULL;

	for (size_t i = 0; i < ARRAY_SIZE(keyname_to_icode_map); i++) {
		if (keyname_to_icode_map[i].icode == icode) {
			prefix = "KEY_";
			keyname = keyname_to_icode_map[i].name;
		}
	}

	if (keyname == NULL) {
		for (size_t i = 0; i < ARRAY_SIZE(keypad_to_icode_map); i++) {
			if (keypad_to_icode_map[i].icode == icode) {
				prefix = "KEYPAD_";
				keyname = keypad_to_icode_map[i].name;
			}
		}
	}

	if (keyname == NULL)
		return -ENOENT;

	int ret = snprintf(buf, len, "%s%s", prefix, keyname);
	if ((size_t)ret >= len)
		return -EMSGSIZE;

	return ret;
}

int hm_input_string_to_icode(const char *s)
{
	if (strncmp(s, "KEY_", 4) == 0) {
		for (size_t i = 0; i < ARRAY_SIZE(keyname_to_icode_map); i++) {
			if (strcmp(s + 4, keyname_to_icode_map[i].name) == 0)
				return keyname_to_icode_map[i].icode;
		}
	} else if (strncmp(s, "KEYPAD_", 7) == 0) {
		for (size_t i = 0; i < ARRAY_SIZE(keypad_to_icode_map); i++) {
			if (strcmp(s + 7, keypad_to_icode_map[i].name) == 0)
				return keypad_to_icode_map[i].icode;
		}
	}

	LOG_WRN("Failed to convert keystring <%s> to code", s);
	return -EINVAL;
}

// NOTE: This function modfies the passed string, so make sure it is not needed
// or make a copy beforehand.
int hm_input_icode_list_from_string(char *str, uint8_t *out, size_t len)
{
	char *rest = str;
	char *token;

	size_t i = 0;
	while ((token = strtok_r(rest, " ", &rest)) && i < len) {
		int ret = hm_input_string_to_icode(token);

		if (ret < 0)
			continue;

		out[i] = ret;
		i++;
	}

	return i;
}

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
static atomic_ptr_t current_kbd_layout = (atomic_ptr_t)&kbd_layout_table[2];

const char *hm_input_get_current_kbd_layout_locale(void)
{
	const struct kbd_layout_entry *layout = (const struct kbd_layout_entry *)atomic_ptr_get(&current_kbd_layout);
	return layout->locale;
}

int hm_input_set_kbd_layout_locale(const char *locale)
{
	for (size_t i = 0; i < ARRAY_SIZE(kbd_layout_table); i++) {
		if (strcmp(locale, kbd_layout_table[i].locale) == 0) {
			atomic_ptr_set(&current_kbd_layout, (atomic_ptr_t)&kbd_layout_table[i]);
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

	const struct kbd_layout_entry *layout = (const struct kbd_layout_entry *)atomic_ptr_get(&current_kbd_layout);
	uint8_t k = layout->layout[c];

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

int hm_input_icode_to_hid(uint8_t icode)
{
	if (icode >= 0x10 && icode <= 0x17) {
		return icode - 0x10 + 0xE0;
	} else if (icode > 0x80) {
		return icode - 0x80;
	} else {
		struct hid_keystroke keystroke = ascii_to_hid_keystroke(icode);
		return keystroke.code;
	}

	return 0;
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
		}

		if (keystroke.altgr != altgr_active) {
			altgr_active = keystroke.altgr;
			hm_input_report_key(HID_KEY_MOD_RALT, altgr_active, K_NO_WAIT);
		}

		hm_input_report_key(keystroke.code, true, K_NO_WAIT);
		hm_input_report_key(keystroke.code, false, K_NO_WAIT);

		s++;

	}

	if (shift_active)
		hm_input_report_key(HID_KEY_MOD_LSHIFT, false, K_NO_WAIT);

	if (altgr_active)
		hm_input_report_key(HID_KEY_MOD_RALT, false, K_NO_WAIT);

	return 0;
}
