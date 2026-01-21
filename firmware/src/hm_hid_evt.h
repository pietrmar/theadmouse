#pragma once

#include <stdint.h>

// For HID_KEY_* definitions
#include <zephyr/usb/class/hid.h>

#define HID_KEY_MOD_LCTRL	0xE0
#define HID_KEY_MOD_LSHIFT	0xE1
#define HID_KEY_MOD_LALT	0xE2
#define HID_KEY_MOD_LGUI	0xE3
#define HID_KEY_MOD_RCTRL	0xE4
#define HID_KEY_MOD_RSHIFT	0xE5
#define HID_KEY_MOD_RALT	0xE6
#define HID_KEY_MOD_RGUI	0xE7

enum hm_hid_evt_type {
	HM_HID_EVT_KEY,

	HM_HID_EVT_MOUSE_MOVE,
	HM_HID_EVT_MOUSE_WHEEL,
	HM_HID_EVT_MOUSE_BTN,
};

struct hm_hid_evt_key {
	uint8_t hid_keycode;
	bool pressed;
};

struct hm_hid_evt_mouse_move {
	int8_t dx;
	int8_t dy;
};

struct hm_hid_evt_mouse_wheel {
	int8_t steps;
};

enum hm_hid_mouse_btn {
	HM_MOUSE_BTN_LEFT	= 0,
	HM_MOUSE_BTN_RIGHT	= 1,
	HM_MOUSE_BTN_MIDDLE	= 2,
};

struct hm_hid_evt_mouse_btn {
	enum hm_hid_mouse_btn btn;
	bool pressed;
};

struct hm_hid_evt {
	enum hm_hid_evt_type type;
	union {
		struct hm_hid_evt_key key;
		struct hm_hid_evt_mouse_move mouse_move;
		struct hm_hid_evt_mouse_wheel mouse_wheel;
		struct hm_hid_evt_mouse_btn mouse_btn;
	};
};

