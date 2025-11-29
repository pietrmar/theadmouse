#include <stdint.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/services/hids.h>

#include <zephyr/logging/log.h>

#include "ble_hid_service.h"

LOG_MODULE_REGISTER(ble_hid_service, CONFIG_BLE_HID_SERVICE_LOG_LEVEL);

#define REP_KDB_IDX	0
#define REP_KBD_ID	1
#define REP_KBD_SIZE	8

#define REP_MOUSE_IDX	1
#define REP_MOUSE_ID	2
#define REP_MOUSE_SIZE	4

static const uint8_t hid_report_map[] = {
	// Keyboard descriptor
	0x05, 0x01,	// Usage Page (Generic Desktop)
	0x09, 0x06,	// Usage (Keyboard)
	0xA1, 0x01,	// Collection (Application)
		0x85, REP_KBD_ID,
		0x05, 0x07,	// Usage Page (Keyboard / Keypad)

		// Modifier byte
		0x75, 0x01,	// Report Size (1)
		0x95, 0x08,	// Report Count (8)
		0x19, 0xE0,	// Usage Minimum (Left Ctrl)
		0x29, 0xE7,	// Usage Maximum (Right GUI)
		0x15, 0x00,	// Logical Minimum (0)
		0x25, 0x01,	// Logical Maximum (1)
		0x81, 0x02,	// Input (Data, Var, Abs)

		// Reserved byte
		0x75, 0x08,	// Report Size (8)
		0x95, 0x01,	// Report Count (1)
		0x81, 0x01,	// Input (Const)

		// 6 keycodes
		0x75, 0x08,	// Report Size (8)
		0x95, 0x06,	// Report Count (6)
		0x19, 0x00,	// Usage Minimum (0)
		0x29, 0x65,	// Usage Maximum (101)
		0x15, 0x00,	// Logical Minimum (0)
		0x25, 0x65,	// Logical Maximum (101)
		0x81, 0x00,	// Input (Data, Array, Abs)
	0xC0,		// End Collection (Application)

	// Mouse descriptor
	0x05, 0x01,	// Usage Page (Generic Desktop)
	0x09, 0x02,	// Usage (Mouse)
	0xA1, 0x01,	// Collection (Application)
		0x85, REP_MOUSE_ID,

		0x09, 0x01,	// Usage (Pointer)
		0xA1, 0x00,	// Collection (Physical)
			0x05, 0x09,	// Usage Page (Button)
			0x75, 0x01,	// Report Size (1)
			0x95, 0x03,	// Report Count (3)
			0x19, 0x01,	// Usage Minimum (Button 1)
			0x29, 0x03,	// Usage Maximum (Button 3)
			0x15, 0x00,	// Logical Minimum (0)
			0x25, 0x01,	// Logical Maximum (1)
			0x81, 0x02,	// Input (Data, Var, Abs)

			// 5 bits padding
			0x75, 0x05,	// Report Size (5)
			0x95, 0x01,	// Report Count (1)
			0x81, 0x01,	// Input (Const)

			// X, Y movement
			0x05, 0x01,	// Usage Page (Generic Desktop)
			0x09, 0x30,	// Usage (X)
			0x09, 0x31,	// Usage (Y)
			0x75, 0x08,	// Report Size (8)
			0x95, 0x02,	// Report Count (2)
			0x15, 0x81,	// Logical Minimum (-127)
			0x25, 0x7F,	// Logical Maximum (127)
			0x81, 0x06,	// Input (Data, Var, Rel)

			// Wheel
			0x09, 0x38,	// Usage (Wheel)
			0x75, 0x08,	// Report Size (8)
			0x95, 0x01,	// Report Count (1)
			0x15, 0x81,	// Logical Minimum (-127)
			0x25, 0x7F,	// Logical Maximum (127)
			0x81, 0x06,	// Input (Data, Var, Rel)
		0xC0, 		// End Collection (Physical)
	0xC0,		// End Collection (Application)
};


BT_HIDS_DEF(hids_obj,
	    REP_KBD_SIZE,
	    REP_MOUSE_SIZE);

static struct bt_conn *current_conn = NULL;

static struct bt_hids_init_param hids_init_obj = {
	.info = {
		.bcd_hid = 0x0111,
		.b_country_code = 0,
		.flags = BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE,
	},
	.rep_map = {
		.data = hid_report_map,
		.size = sizeof(hid_report_map),
	},
	.inp_rep_group_init = {
		.reports = {
			[REP_KDB_IDX] 	= { .id = REP_KBD_ID,	.size = REP_KBD_SIZE, },
			[REP_MOUSE_IDX]	= { .id = REP_MOUSE_ID,	.size = REP_MOUSE_SIZE, },
		},
		.cnt = 2,
	},
};

int ble_hid_service_init(void)
{
	LOG_INF("Starting BLE HID service");

	int ret = bt_hids_init(&hids_obj, &hids_init_obj);
	if (ret < 0) {
		LOG_ERR("Failed to init HIDS: %d", ret);
		return ret;
	}

	return 0;
}

int ble_hid_service_connected(struct bt_conn *conn)
{
	if (current_conn != NULL) {
		LOG_ERR("current_conn != NULL on connect");
	}

	current_conn = bt_conn_ref(conn);
	return bt_hids_connected(&hids_obj, conn);
}

int ble_hid_service_disconnected(struct bt_conn *conn)
{
	if (current_conn != conn) {
		LOG_ERR("current_conn does not match on disconnect");
	}

	bt_conn_unref(current_conn);
	current_conn = NULL;
	return bt_hids_disconnected(&hids_obj, conn);
}


#define HM_HID_QUEUE_LEN	64
K_MSGQ_DEFINE(hm_hid_msgq, sizeof(struct hm_hid_evt), HM_HID_QUEUE_LEN, 4);

int ble_hid_queue_evt(struct hm_hid_evt *evt, k_timeout_t timeout)
{
	return k_msgq_put(&hm_hid_msgq, evt, timeout);
}

static inline bool is_modifier(uint8_t hid_keycode)
{
	return hid_keycode >= 0xE0 && hid_keycode <= 0xE7;
}

static uint8_t kbd_state[REP_KBD_SIZE] = { 0 };
static bool apply_key(uint8_t hid_keycode, bool pressed)
{
	bool changed = false;

	if (is_modifier(hid_keycode)) {
		uint8_t mask = 1 << (hid_keycode - 0xE0);

		if (pressed) {
			if (!(kbd_state[0] & mask)) {
				kbd_state[0] |= mask;
				changed = true;
			}
		} else {
			if (kbd_state[0] & mask) {
				kbd_state[0] &= ~mask;
				changed = true;
			}
		}

		return changed;
	}

	ssize_t free_idx = -1;
	ssize_t key_idx = -1;

	for (size_t i = 2; i < sizeof(kbd_state); i++) {
		if (kbd_state[i] == 0 && free_idx == -1)
			free_idx = i;
		if (kbd_state[i] == hid_keycode && key_idx == -1)
			key_idx = i;

		if (free_idx != -1 && key_idx != -1)
			break;
	}

	if (pressed) {
		// TODO: If `key_idx == -1` && `free_idx == -1` then we ran out of space in the
		// report, at least print a log message.
		if (key_idx == -1 && free_idx != -1) {
			kbd_state[free_idx] = hid_keycode;
			changed = true;
		}
	} else {
		if (key_idx != -1) {
			kbd_state[key_idx] = 0;
			changed = true;
		}
	}

	return changed;
}

static int send_kbd_report(const uint8_t buf[REP_KBD_SIZE])
{
	if (current_conn == NULL)
		return -ENOTCONN;

	// TODO: I've not seen it fail, but consider handling `-ENOMEM` when the Tx queue is full
	int ret = bt_hids_inp_rep_send(&hids_obj, current_conn, REP_KDB_IDX, buf, REP_KBD_SIZE, NULL);
	if (ret < 0)
		LOG_ERR("Send kbd report failed: %d", ret);

	return ret;
}

static uint8_t mouse_btn_state = 0;
static bool apply_mouse_btn(uint8_t btn_idx, bool pressed)
{
	uint8_t mask = 1 << btn_idx;
	uint8_t old_state = mouse_btn_state;

	if (pressed)
		mouse_btn_state |= mask;
	else
		mouse_btn_state &= ~mask;

	return old_state != mouse_btn_state;
}

static int send_mouse_report(int8_t dx, int8_t dy, int8_t wheel, uint8_t btn_state)
{
	if (current_conn == NULL)
		return -ENOTCONN;

	uint8_t rep[REP_MOUSE_SIZE];
	rep[0] = btn_state;
	rep[1] = dx;
	rep[2] = dy;
	rep[3] = wheel;

	// TODO: I've not seen it fail, but consider handling `-ENOMEM` when the Tx queue is full
	int ret = bt_hids_inp_rep_send(&hids_obj, current_conn, REP_MOUSE_IDX, rep, REP_MOUSE_SIZE, NULL);
	if (ret < 0)
		LOG_ERR("Send mouse report failed: %d", ret);

	return ret;
}



static void ble_hid_thread(void *p1, void *p2, void *p3)
{
	while (true) {
		struct hm_hid_evt evt;
		k_msgq_get(&hm_hid_msgq, &evt, K_FOREVER);

		do {
			switch (evt.type) {
				case HM_HID_EVT_KEY: {
					bool changed = apply_key(evt.key.hid_keycode, evt.key.pressed);

					if (changed)
						send_kbd_report(kbd_state);

					break;
				}
				case HM_HID_EVT_MOUSE_MOVE: {
					int8_t dx = evt.mouse_move.dx;
					int8_t dy = evt.mouse_move.dy;

					if (dx != 0 || dy != 0)
						send_mouse_report(dx, dy, 0, mouse_btn_state);

					break;
				}
				case HM_HID_EVT_MOUSE_WHEEL: {
					int8_t w = evt.mouse_wheel.steps;

					if (w != 0)
						send_mouse_report(0, 0, w, mouse_btn_state);

					break;
				}
				case HM_HID_EVT_MOUSE_BTN: {
					bool changed = apply_mouse_btn(evt.mouse_btn.btn, evt.mouse_btn.pressed);

					if (changed)
						send_mouse_report(0, 0, 0, mouse_btn_state);

					break;
				}
				default:
					LOG_WRN("Unhandled HID queue message type: %d", evt.type);
					break;
			}
		} while (k_msgq_get(&hm_hid_msgq, &evt, K_NO_WAIT) == 0);
	}
}
K_THREAD_DEFINE(ble_hid_tid, 2048, ble_hid_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, 0);
