
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <ctype.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/app_version.h>
#include <zephyr/logging/log.h>

#include "common.h"
#include "at_cmd.h"
#include "at_cmd_param.h"
#include "cmd_uart.h"
#include "button_manager.h"
#include "slot_manager.h"
#include "motion_engine.h"
#include "headmouse_input.h"

LOG_MODULE_REGISTER(at, CONFIG_AT_LOG_LEVEL);

static int at_cmd_ID(const struct at_cmd_param *arg, void *ctx)
{
	// TODO: A current FABI prints the following, so make sure to extend it:
	// FABI v3.7.1, PressureSensor=None, ForceSensor=None, Board=Raspberry_Pi_Pico_2W, VID=0x2E8A, PID=0xF10F, TotalBytes=1048576, UsedBytes=28672, FreeBytes=1019904, MemoryUsed=2%
	int ret = at_replyf("%s %s PressureSensor=None ForceSensor=IMU", CONFIG_APP_PROJECT_NAME, APP_VERSION_STRING);
	if (ret < 0)
		return ret;

	return 0;
}

static int at_cmd_MM(const struct at_cmd_param *arg, void *ctx)
{
	uint32_t p;

	int ret = at_cmd_param_get_uint(arg, &p);
	if (ret < 0)
		return ret;

	return motion_engine_set_mouse_mode((enum mouse_mode)p);
}

static int at_cmd_WA(const struct at_cmd_param *arg, void *ctx)
{
	uint32_t ms;

	int ret = at_cmd_param_get_uint(arg, &ms);
	if (ret < 0)
		return ret;

	if (ms > 1000) {
		ms = 1000;
		LOG_WRN("Clamping WA wait time to 1000 ms");
	}

	k_sleep(K_MSEC(ms));

	return 0;
}

static int at_cmd_NC(const struct at_cmd_param *arg, void *ctx)
{
	return 0;
}

static int at_cmd_SC(const struct at_cmd_param *arg, void *ctx)
{
	uint32_t col;

	int ret = at_cmd_param_get_uint(arg, &col);
	if (ret < 0)
		return ret;

	return slot_manager_set_color(col);
}


static struct k_poll_signal generate_report_signal = K_POLL_SIGNAL_INITIALIZER(generate_report_signal);

static void report_timer_expiry_fn(struct k_timer *timer)
{
	k_poll_signal_raise(&generate_report_signal, 1);
}
K_TIMER_DEFINE(report_timer, report_timer_expiry_fn, NULL);

static int at_cmd_xR(const struct at_cmd_param *arg, void *ctx)
{
	bool enable = (bool)ctx;

	if (enable) {
		k_timer_start(&report_timer, K_MSEC(0), K_MSEC(50));
	} else {
		k_timer_stop(&report_timer);
		k_poll_signal_reset(&generate_report_signal);
	}

	return 0;
}

static int at_cmd_CA(const struct at_cmd_param *arg, void *ctx)
{
	motion_engine_reset_absolute_pos();
	return 0;
}

static int at_cmd_NE(const struct at_cmd_param *arg, void *ctx)
{
	// NOTE: The slot manager will directly dispatch AT commands
	// bypassing the queue.
	return slot_manager_load_next_slot();
}

static int at_cmd_EC(const struct at_cmd_param *arg, void *ctx)
{
	const char *s;

	int ret = at_cmd_param_get_str(arg, &s);
	if (ret < 0)
		return ret;

	LOG_INF("AT echo: %s", s);

	ret = at_replyf("%s", s);
	if (ret < 0)
		return ret;

	return 0;
}

static int at_cmd_SA(const struct at_cmd_param *arg, void *ctx)
{
	const char *s;

	int ret = at_cmd_param_get_str(arg, &s);
	if (ret < 0)
		return ret;


	ret = slot_manager_save_current_slot_by_name(s);
	if (ret < 0)
		return ret;

	at_reply("OK");
	return 0;
}

static int at_cmd_LO(const struct at_cmd_param *arg, void *ctx)
{
	const char *s;

	int ret = at_cmd_param_get_str(arg, &s);
	if (ret < 0)
		return ret;

	// NOTE: The slot manager will directly dispatch AT commands
	// bypassing the queue.
	ret = slot_manager_load_slot_by_name(s);
	if (ret < 0)
		return ret;

	at_reply("OK");
	return 0;
}

static int at_cmd_LA(const struct at_cmd_param *arg, void *ctx)
{
	return slot_manager_dump_all_slots(at_replyf);
}

static int at_cmd_LI(const struct at_cmd_param *arg, void *ctx)
{
	int ret = slot_manager_list_all_slots(at_replyf);
	if (ret < 0) {
		return ret;
	}

	at_reply("OK");
	return 0;
}

struct mouse_click_params {
	enum hm_hid_mouse_btn btn;
	bool double_click;
};
static int at_cmd_Cx(const struct at_cmd_param *arg, void *ctx)
{
	struct mouse_click_params *params = (struct mouse_click_params *)ctx;

	int ret = 0;
	ret = hm_input_report_mouse_btn(params->btn, true, K_FOREVER);
	if (ret < 0)
		return ret;

	ret = hm_input_report_mouse_btn(params->btn, false, K_FOREVER);
	if (ret < 0)
		return ret;

	if (params->double_click) {
		// TODO: Consider scheduling the second click via a delayed work
		// to not do an actual sleep here. Not sure if we want this, think
		// about it.
		k_sleep(K_MSEC(100));

		ret = hm_input_report_mouse_btn(params->btn, true, K_FOREVER);
		if (ret < 0)
			return ret;

		ret = hm_input_report_mouse_btn(params->btn, false, K_FOREVER);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int at_cmd_KW(const struct at_cmd_param *arg, void *ctx)
{
	const char *s;

	int ret = at_cmd_param_get_str(arg, &s);
	if (ret < 0)
		return ret;

	return hm_input_write_string(s);
}

static int at_cmd_KL(const struct at_cmd_param *arg, void *ctx)
{
	const char *s;

	// If no parameter was passed then just print the current locale
	if (at_cmd_param_get_type(arg) == AT_CMD_PARAM_TYPE_NONE)
		return at_reply(hm_input_get_current_kbd_layout_locale());

	int ret = at_cmd_param_get_str(arg, &s);
	if (ret < 0)
		return ret;

	ret = hm_input_set_kbd_layout_locale(s);
	if (ret < 0)
		LOG_WRN("Failed to set keyboard layout to %s: %d", s, ret);

	return ret;
}

enum mouse_wheel {
	MOUSE_WHEEL_UP,
	MOUSE_WHEEL_DOWN,
};
static int at_cmd_Wx(const struct at_cmd_param *arg, void *ctx)
{
	enum mouse_wheel wheel = (enum mouse_wheel)(uintptr_t)ctx;

	// TODO: Take the step count from an `AT WS` command
	int8_t steps = wheel == MOUSE_WHEEL_UP ? +10 : -10;

	return hm_input_report_mouse_wheel(steps, K_FOREVER);
}

static int at_cmd_Mx(const struct at_cmd_param *arg, void *ctx)
{
	int d;
	enum axis axis = (enum axis)(uintptr_t)ctx;

	int ret = at_cmd_param_get_int(arg, &d);
	if (ret < 0)
		return ret;

	int16_t dx = axis == AXIS_X ? d : 0;
	int16_t dy = axis == AXIS_Y ? d : 0;

	return hm_input_report_mouse_move(dx, dy, K_FOREVER);
}

static int at_cmd_Ax(const struct at_cmd_param *arg, void *ctx)
{
	uint32_t ui;
	enum axis axis = (enum axis)(uintptr_t)ctx;

	int ret = at_cmd_param_get_uint(arg, &ui);
	if (ret < 0)
		return ret;

	return motion_engine_set_acceleration(ui, axis);
}

static int at_cmd_Dx(const struct at_cmd_param *arg, void *ctx)
{
	uint32_t ui;
	enum axis axis = (enum axis)(uintptr_t)ctx;

	int ret = at_cmd_param_get_uint(arg, &ui);
	if (ret < 0)
		return ret;

	return motion_engine_set_deadzone(ui, axis);
}

// TODO: Instead of handling this ourselved here, add a simple `button_manager_arm()` or so
// API to separate the concerns better and also check against other things. This could also
// move the size/index check into the button_manager.
static int32_t set_slot_command = -1;
static int at_cmd_BM(const struct at_cmd_param *arg, void *ctx)
{
	uint32_t btn_idx;

	int ret = at_cmd_param_get_uint(arg, &btn_idx);
	if (ret < 0)
		return ret;

	// For the end user the index starts at 1, but internally it
	// starts at 0, so we need to check this here.
	if (btn_idx < 1 || !button_manager_valid_index(btn_idx - 1)) {
		LOG_ERR("Button index %u is out of range", btn_idx);
		return -EINVAL;
	}

	set_slot_command = btn_idx - 1;

	return 0;
}

static int at_cmd_internal_empty_command(const struct at_cmd_param *arg, void *ctx)
{
	at_reply("OK");
	return 0;
}

static int at_cmd_internal_load_slot_index(const struct at_cmd_param *arg, void *ctx)
{
	uint32_t idx;

	int ret = at_cmd_param_get_uint(arg, &idx);
	if (ret < 0)
		return ret;

	return slot_manager_load_slot_by_index(idx);
}

static const struct at_cmd at_cmds[] = {
	{ MAKE2CC(ID), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_ID, NULL },

	{ MAKE2CC(MM), 0, AT_CMD_PARAM_TYPE_UINT, at_cmd_MM, NULL },
	{ MAKE2CC(WA), 0, AT_CMD_PARAM_TYPE_UINT, at_cmd_WA, NULL },
	{ MAKE2CC(NC), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_NC, NULL },

	{ MAKE2CC(CL), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_Cx, &(struct mouse_click_params){ .btn = HM_MOUSE_BTN_LEFT, .double_click = false } },
	{ MAKE2CC(CD), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_Cx, &(struct mouse_click_params){ .btn = HM_MOUSE_BTN_LEFT, .double_click = true } },
	{ MAKE2CC(CR), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_Cx, &(struct mouse_click_params){ .btn = HM_MOUSE_BTN_RIGHT, .double_click = false } },
	{ MAKE2CC(CM), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_Cx, &(struct mouse_click_params){ .btn = HM_MOUSE_BTN_MIDDLE, .double_click = false } },

	{ MAKE2CC(WU), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_Wx, (void *)MOUSE_WHEEL_UP },
	{ MAKE2CC(WD), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_Wx, (void *)MOUSE_WHEEL_DOWN },

	{ MAKE2CC(MX), 0, AT_CMD_PARAM_TYPE_INT, at_cmd_Mx, (void *)AXIS_X },
	{ MAKE2CC(MY), 0, AT_CMD_PARAM_TYPE_INT, at_cmd_Mx, (void *)AXIS_Y },

	{ MAKE2CC(KW), 0, AT_CMD_PARAM_TYPE_STR, at_cmd_KW, NULL },
	{ MAKE2CC(KL), AT_CMD_FLAG_OPT_PARAM, AT_CMD_PARAM_TYPE_STR, at_cmd_KL, NULL },

	{ MAKE2CC(BM), 0, AT_CMD_PARAM_TYPE_UINT, at_cmd_BM, NULL },

	{ MAKE2CC(SC), 0, AT_CMD_PARAM_TYPE_UINT, at_cmd_SC, NULL },
	{ MAKE2CC(SR), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_xR, (void *)true },
	{ MAKE2CC(ER), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_xR, (void *)false },

	{ MAKE2CC(AX), 0, AT_CMD_PARAM_TYPE_UINT, at_cmd_Ax, (void *)AXIS_X },
	{ MAKE2CC(AY), 0, AT_CMD_PARAM_TYPE_UINT, at_cmd_Ax, (void *)AXIS_Y },
	{ MAKE2CC(DX), 0, AT_CMD_PARAM_TYPE_UINT, at_cmd_Dx, (void *)AXIS_X },
	{ MAKE2CC(DY), 0, AT_CMD_PARAM_TYPE_UINT, at_cmd_Dx, (void *)AXIS_Y },
	{ MAKE2CC(CA), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_CA, NULL },

	{ MAKE2CC(SA), 0, AT_CMD_PARAM_TYPE_STR, at_cmd_SA, NULL },
	{ MAKE2CC(LO), 0, AT_CMD_PARAM_TYPE_STR, at_cmd_LO, NULL },
	{ MAKE2CC(LA), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_LA, NULL },
	{ MAKE2CC(LI), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_LI, NULL },
	{ MAKE2CC(NE), 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_NE, NULL },

	// Echo command, not in the original FABI, but added here for debugging purposes
	{ MAKE2CC(EC), 0, AT_CMD_PARAM_TYPE_STR, at_cmd_EC, NULL },

	{ AT_CMD_INTERNAL_EMPTY_COMMAND, 0, AT_CMD_PARAM_TYPE_NONE, at_cmd_internal_empty_command, NULL },
	{ AT_CMD_INTERNAL_LOAD_SLOT_INDEX, 0, AT_CMD_PARAM_TYPE_UINT, at_cmd_internal_load_slot_index, NULL },
};

static const struct at_cmd *find_at_cmd(const uint16_t code)
{
	for (int i = 0; i < ARRAY_SIZE(at_cmds); i++) {
		if (at_cmds[i].code == code) {
			return &at_cmds[i];
		}
	}

	return NULL;
}

static inline char *ltrim(char *p)
{
	while (*p != '\0') {
		if (!isspace(*p)) {
			break;
		}
		p++;
	}

	return p;
}

static inline char *rtrim(char *p)
{
	size_t l = strlen(p);

	while (l > 0) {
		if (!isspace(p[l-1])) {
			break;
		}
		l--;
	}

	p[l] = '\0';
	return p;
}

struct at_cmd_work_item {
	const struct at_cmd *cmd;
	struct at_cmd_param param;

	struct k_sem *done_sem;
	int *result_ptr;
};

#define AT_CMD_QUEUE_DEPTH 8
K_MSGQ_DEFINE(at_cmd_queue, sizeof(struct at_cmd_work_item), AT_CMD_QUEUE_DEPTH, 4);

// This does no error checking and assumes that `cmd` and `cmd->cb` is valid
static int __at_cmd_dispatch_ptr(const struct at_cmd *cmd, const struct at_cmd_param *param)
{
	// TODO: Use some kind of "arming" API or so
	// TODO: We probably want to disallow certain commands to be mapped on buttons
	if (set_slot_command != -1) {
		int ret = button_manager_set_mapping(set_slot_command, cmd->code, param);

		if (ret < 0)
			LOG_ERR("Failed to set button mapping for slot %d: %d", set_slot_command, ret);

		set_slot_command = -1;
		return ret;
	}

	return cmd->cb(param, cmd->ctx);
}


static struct k_poll_event at_cmd_thread_poll_events[2] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &at_cmd_queue, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &generate_report_signal, 0),
};

static int at_cmd_process_next(struct k_msgq *msgq)
{
	struct at_cmd_work_item work_item;

	int ret = k_msgq_get(msgq, &work_item, K_NO_WAIT);
	if (ret < 0)
		return ret;

	if (work_item.cmd->cb != NULL) {
		char buf[3];
		LOG_DBG("Dispatching <%s>", at_code_to_str(work_item.cmd->code, buf));

		ret = __at_cmd_dispatch_ptr(work_item.cmd, &work_item.param);
		if (ret < 0)
			LOG_DBG("AT command <%s> failed: %d", at_code_to_str(work_item.cmd->code, buf), ret);
	} else {
		char buf[3];
		LOG_WRN("Callback for AT command <%s> not implemented", at_code_to_str(work_item.cmd->code, buf));
		ret = -ENOTSUP;
	}

	// TODO: If `param->flags` & AT_CMD_PARAM_FLAG_HEAPSTR` then `at_cmd_enqueue_*()` allocated and made a copy
	// of the string which now needs to be freed.

	if (work_item.result_ptr)
		*work_item.result_ptr = ret;

	if (work_item.done_sem)
		k_sem_give(work_item.done_sem);

	// The function and processing of the command is always successful, and error codes from the actual
	// AT commands are returned in the `.result_ptr` pointer to the calling path.
	return 0;
}

static void at_cmd_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	LOG_INF("Started AT dispatch thread");

	while (true) {
		int ret = k_poll(at_cmd_thread_poll_events, ARRAY_SIZE(at_cmd_thread_poll_events), K_FOREVER);

		if (ret < 0) {
			LOG_WRN("k_poll() failed: %d", ret);
			continue;
		}

		if (at_cmd_thread_poll_events[0].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
			while (at_cmd_process_next(at_cmd_thread_poll_events[0].msgq) == 0) {
				// Keep going until the queue is empty
			}

			at_cmd_thread_poll_events[0].state = K_POLL_STATE_NOT_READY;
		}

		if (at_cmd_thread_poll_events[1].state == K_POLL_STATE_SIGNALED) {
			int signaled, result;
			k_poll_signal_check(at_cmd_thread_poll_events[1].signal, &signaled, &result);
			k_poll_signal_reset(at_cmd_thread_poll_events[1].signal);
			at_cmd_thread_poll_events[1].state = K_POLL_STATE_NOT_READY;

			if (signaled && (result == 1)) {
				char buf[128];

				float raw_x, raw_y;
				ret = motion_engine_get_absolute_pos(&raw_x, &raw_y);
				if (ret < 0) {
					LOG_ERR("Failed to get absolute position from motion engine: %d", ret);
					raw_x = 0.0f;
					raw_y = 0.0f;
				}

				int i_raw_x = (int)roundf(raw_x);
				int i_raw_y = (int)roundf(raw_y);

				// Currentlty expected format by the WebGUI:
				//  VALUES:<pressure>,<down>,<up>,<right>,<left>,<x-raw>,<y-raw>,<buttons>,<slot>
				at_replyf("VALUES:0,0,0,0,0,%d,%d,%s,%d",
						i_raw_x, i_raw_y,
						button_manager_get_button_state_string(buf, sizeof(buf)),
						slot_manager_get_active_slot_idx());
			}
		}
	}
}
K_THREAD_DEFINE(at_cmd_tid, 2048, at_cmd_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, 0);

// TODO: This currently only accepts K_FOREVER, if we want actually support timeouts then we need
// to ensure the lifetime of `done_sem` and `result` in case we return earlier when a timeout is
// reached.
int at_cmd_enqueue_and_wait_ptr(const struct at_cmd *cmd, const struct at_cmd_param *param, k_timeout_t timeout)
{
	if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		LOG_ERR("Enqueue and wait not allowed with K_NO_WAIT");
		return -EWOULDBLOCK;
	}

	if (!K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		LOG_ERR("Enqueue and wait only supported with K_FOREVER");
		return -ENOTSUP;
	}

	struct k_sem done_sem;
	k_sem_init(&done_sem, 0, 1);
	int result;
	struct at_cmd_work_item work_item = {
		.cmd = cmd,
		.param = *param,
		.done_sem = &done_sem,
		.result_ptr = &result,
	};

	// TODO: If `param->flags` & AT_CMD_PARAM_FLAG_HEAPSTR` then make a slab allocation here and
	// copy the heap-string. It should be freed once the command has been processed.

	int ret = k_msgq_put(&at_cmd_queue, &work_item, timeout);
	if (ret < 0) {
		char buf[3];
		LOG_ERR("Failed to enqueue AT command <%s>: %d", at_code_to_str(cmd->code, buf), ret);
		return ret;
	}

	ret = k_sem_take(&done_sem, timeout);
	if (ret < 0)
		return ret;

	return result;
}

int at_cmd_enqueue_ptr(const struct at_cmd *cmd, const struct at_cmd_param *param, k_timeout_t timeout)
{
	struct at_cmd_work_item work_item = {
		.cmd = cmd,
		.param = *param,
		.done_sem = NULL,
		.result_ptr = NULL,
	};

	// TODO: If `param->flags` & AT_CMD_PARAM_FLAG_HEAPSTR` then make a slab allocation here and
	// copy the heap-string. It should be freed once the command has been processed.

	int ret = k_msgq_put(&at_cmd_queue, &work_item, timeout);
	if (ret < 0) {
		char buf[3];
		LOG_ERR("Failed to enqueue AT command <%s>: %d", at_code_to_str(cmd->code, buf), ret);
		return ret;
	}

	return 0;
}

int at_cmd_enqueue_and_wait_code(const uint16_t code, const struct at_cmd_param *param, k_timeout_t timeout)
{
	const struct at_cmd *cmd = find_at_cmd(code);
	if (cmd == NULL) {
		char buf[3];
		LOG_ERR("Could not find AT cmd <%s>", at_code_to_str(code, buf));
		return -ENOTSUP;
	}

	return at_cmd_enqueue_and_wait_ptr(cmd, param, timeout);
}

int at_cmd_enqueue_code(const uint16_t code, const struct at_cmd_param *param, k_timeout_t timeout)
{
	const struct at_cmd *cmd = find_at_cmd(code);
	if (cmd == NULL) {
		char buf[3];
		LOG_ERR("Could not find AT cmd <%s>", at_code_to_str(code, buf));
		return -ENOTSUP;
	}

	return at_cmd_enqueue_ptr(cmd, param, timeout);
}

int at_cmd_dispatch_ptr(const struct at_cmd *cmd, const struct at_cmd_param *param)
{
	char buf[3];

	if (k_current_get() != at_cmd_tid) {
		LOG_ERR("%s must only be called from the AT dispatcher thread", __func__);
		return -EPERM;
	}

	if (!cmd->cb) {
		// Do not fail here fully but print a warning
		LOG_WRN("Callback for AT command <%s> not implemented", at_code_to_str(cmd->code, buf));
		return 0;
	}

	LOG_DBG("Directly dispatching <%s>", at_code_to_str(cmd->code, buf));
	return __at_cmd_dispatch_ptr(cmd, param);
}

int at_cmd_dispatch_code(const uint16_t code, const struct at_cmd_param *param)
{
	char buf[3];
	const struct at_cmd *cmd = find_at_cmd(code);
	if (cmd == NULL) {
		LOG_ERR("Could not find AT cmd <%s>", at_code_to_str(code, buf));
		return -ENOTSUP;
	}

	return at_cmd_dispatch_ptr(cmd, param);
}

// NOTE: If we just got a plain AT command without any parameters or so then this will return NULL and `*out_cmd = NULL`.
// NOTE: The assigned `*out_cmd_param` is not deep copied, so if you nede to keep it alive make sure to use `at_cmd_param_clone()`
// TODO: Maybe add a flag like `AT_FLAG_CLONE_PARAMS` that makes a deep-copy if needed.
int at_parse_line_inplace(char *s, const struct at_cmd **out_cmd, struct at_cmd_param *out_cmd_param, enum at_parser_flags flags)
{
	if (!s || !out_cmd || !out_cmd_param)
		return -EINVAL;

	// Remove any trailing and leading whitespace
	s = ltrim(s);
	s = rtrim(s);

	if (*s == '\0')
		return -EINVAL;

	if (strncasecmp(s, "AT", 2) == 0) {
		s += 2;

		// Empty command with just the AT prefix, return the special command here.
		if (*s == '\0') {
			const struct at_cmd *cmd = find_at_cmd(AT_CMD_INTERNAL_EMPTY_COMMAND);
			struct at_cmd_param param = { 0 };

			if (cmd == NULL) {
				LOG_ERR("Missing iternal empty command");
				return -EFAULT;
			}

			*out_cmd_param = param;
			*out_cmd = cmd;
			return 0;
		}

		// The next character following AT must be at least one space or tab.
		// If this is not the case then there is some error
		if (*s != ' ' && *s != '\t')
			return -EINVAL;

		// Trim possible multiple whitespace characters after the AT command
		s = ltrim(s);
	} else {
		// No AT-prefix found, check if allowed by flag
		if (!(flags & AT_PARSER_FLAG_ALLOW_NO_PREFIX))
			return -EINVAL;
	}

	// This will be the start of our command
	char *command = s;
	char *param = NULL;

	// Find the end of the command
	while (*s != '\0' && *s != ' ' && *s != '\t')
		s++;

	// Check if this is the end, otherwise extract the parameter string
	if (*s == '\0') {
		param = NULL;
		LOG_DBG("cmd: <%s>", command);
	} else {
		*s = '\0';

		s++;
		s = ltrim(s);

		if (*s != '\0') {
			param = s;
		}

		LOG_DBG("cmd: <%s>, param: <%s>", command, param);
	}

	size_t cmdlen = strlen(command);
	if (cmdlen != 2) {
		LOG_ERR("Unexpected AT code <%s> length, got: %zu, expected: 3", command, cmdlen);
		return -EINVAL;
	}

	// Find the command
	const struct at_cmd *cmd = find_at_cmd(at_code_from_str(command));
	if (cmd == NULL) {
		LOG_ERR("Could not find AT cmd <%s>", command);
		return -ENOTSUP;
	}

	// Check and print a warning but do not fail if we have extra parameters
	if (cmd->param_type == AT_CMD_PARAM_TYPE_NONE && param != NULL) {
		LOG_WRN("AT cmd <%s> does not take parameters, ignoring <%s>", command, param);
	}

	// Fail if a parameter is missing and is not optional
	if (cmd->param_type != AT_CMD_PARAM_TYPE_NONE && param == NULL && !(cmd->flags & AT_CMD_FLAG_OPT_PARAM)) {
		LOG_WRN("AT cmd <%s> requires a parameter", command);
		return -EINVAL;
	}

	struct at_cmd_param cmd_param = { 0 };

	if (param != NULL) {
		// TODO: Consider not using automatic radix detection when using `strtol()`
		int ret = 0;

		switch (cmd->param_type) {
			case AT_CMD_PARAM_TYPE_NONE:
				break;
			case AT_CMD_PARAM_TYPE_INT:
				ret = at_cmd_param_set_int(&cmd_param, strtol(param, NULL, 0));
				break;
			case AT_CMD_PARAM_TYPE_UINT:
				// Do not allow negative numbers
				if (*param == '-')
					return -EINVAL;

				ret = at_cmd_param_set_uint(&cmd_param, strtoul(param, NULL, 0));
				break;
			case AT_CMD_PARAM_TYPE_STR:
				ret = at_cmd_param_set_str(&cmd_param, param);
				break;
			case AT_CMD_PARAM_TYPE_KEYS:
				uint8_t keys[AT_CMD_PARAM_MAX_KEYS] = { 0 };
				ret = hm_input_icode_list_from_string(param, keys, ARRAY_SIZE(keys));
				if (ret < 0)
					return ret;

				int len = ret;
				ret = at_cmd_param_set_keys(&cmd_param, keys, len);
				break;
			default:
				LOG_WRN("Unhandled param type %d", cmd->param_type);
				return -EINVAL;
				break;
		};

		if (ret < 0)
			return ret;
	}

	*out_cmd = cmd;
	*out_cmd_param = cmd_param;

	return 0;
}

int at_parse_line_copy(const char *s, const struct at_cmd **out_cmd, struct at_cmd_param *out_cmd_param, enum at_parser_flags flags)
{
	if (!s || !*s || !out_cmd || !out_cmd_param)
		return -EINVAL;

	size_t len = strlen(s);
	if (len > AT_LINE_MAX)
		return -EMSGSIZE;

	char buf[AT_LINE_MAX + 1];
	memcpy(buf, s, len + 1);

	return at_parse_line_inplace(buf, out_cmd, out_cmd_param, flags);
}

// TODO: How to handle the formatting of `AT_CMD_PARAM_UINT` if we want a different radix?
int at_format_cmd(char *out_buf, size_t buf_len, const uint16_t code, const struct at_cmd_param *param)
{
	int ret;

	if (!out_buf || buf_len == 0 || !param) {
		return -EINVAL;
	}

	enum at_cmd_param_type param_type = at_cmd_param_get_type(param);
	// TODO: Just to be sure make sure that `code` does not have `AT_CMD_INTERNAL_BIT` set
	// TODO: Check if `code` is actually in our table and check if the type form the table matches `param->type`
	char atbuf[3];
	switch (param_type) {
		case AT_CMD_PARAM_TYPE_NONE:
			ret = snprintf(out_buf, buf_len, "AT %s", at_code_to_str(code, atbuf));
			break;
		case AT_CMD_PARAM_TYPE_INT:
			int32_t i;
			ret = at_cmd_param_get_int(param, &i);
			if (ret < 0)
				return ret;
			ret = snprintf(out_buf, buf_len, "AT %s %d", at_code_to_str(code, atbuf), i);
			break;
		case AT_CMD_PARAM_TYPE_UINT:
			uint32_t ui;
			ret = at_cmd_param_get_uint(param, &ui);
			if (ret < 0)
				return ret;
			ret = snprintf(out_buf, buf_len, "AT %s %u", at_code_to_str(code, atbuf), ui);
			break;
		case AT_CMD_PARAM_TYPE_STR:
			const char *s;
			ret = at_cmd_param_get_str(param, &s);
			if (ret < 0)
				return ret;
			ret = snprintf(out_buf, buf_len, "AT %s %s", at_code_to_str(code, atbuf), s);
			break;
		case AT_CMD_PARAM_TYPE_KEYS:
			const uint8_t *keys;
			ret = at_cmd_param_get_keys(param, &keys);
			if (ret < 0)
				break;

			int keys_len = ret;

			size_t offset = 0;
			ret = snprintf(out_buf, buf_len, "AT %s", at_code_to_str(code, atbuf));
			if (ret < 0 || (size_t)ret >= buf_len)
				break;

			offset = ret;
			for (int i = 0; i < keys_len; i++) {
				if (keys[i] == 0)
					break;

				size_t remaining = (offset < buf_len) ? (buf_len - offset) : 0;

				if (remaining < 2) {
					ret = -EMSGSIZE;
					break;
				}

				out_buf[offset++] = ' ';
				remaining--;
				out_buf[offset] = '\0';

				ret = hm_input_icode_to_keyname(out_buf + offset, remaining, keys[i]);
				if (ret < 0)
					break;

				offset += ret;
			}
			break;
		default:
			LOG_WRN("Unhandled param type %d", param_type);
			return -EINVAL;
			break;
	}

	if (ret < 0 && ret != -EMSGSIZE) {
		LOG_ERR("Failed to format AT command <%s>: %d", at_code_to_str(code, atbuf), ret);
		return ret;
	}

	if ((ret >= 0 && (size_t)ret >= buf_len) || ret == -EMSGSIZE) {
		LOG_ERR("AT format buffer overflow, cmd: <%s> (partial: <%s>)", at_code_to_str(code, atbuf), out_buf);
		return -EMSGSIZE;
	}

	LOG_DBG("Formatted AT command: <%s>", out_buf);

	return 0;
}

int at_handle_line_inplace(char *s, enum at_parser_flags flags)
{
	const struct at_cmd *cmd = NULL;
	struct at_cmd_param cmd_param = { 0 };

	int ret = at_parse_line_inplace(s, &cmd, &cmd_param, flags);
	if (ret < 0) {
		// Since we used the `_inplace()` variant the buffer was
		// modified so we cannot print the whole line here anymore
		LOG_ERR("Failed to parse line: %d", ret);
		return ret;
	}

	return at_cmd_enqueue_ptr(cmd, &cmd_param, K_FOREVER);
}

int at_handle_line_copy(const char *s, enum at_parser_flags flags)
{
	if (!s || !*s)
		return -EINVAL;

	size_t len = strlen(s);
	if (len > AT_LINE_MAX)
		return -EMSGSIZE;

	char buf[AT_LINE_MAX + 1];
	memcpy(buf, s, len + 1);

	return at_handle_line_inplace(buf, flags);
}


// TODO: The `at_putn()`/`at_reply*()` functions should only be used from AT command
// callbacks.
// TODO: We should execute callbacks in a single thread from a queue which
// will ensure that no `at_putn()`/`at_reply*()` calls are interleaved.
static inline int at_putn(const char *s, size_t len)
{
	if (!s) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	ssize_t written = cmd_uart_write((const uint8_t *)s, len);

	if (written < 0) {
		return (int)written;
	}

	if (written != (ssize_t)len) {
		return -EIO;
	}

	return 0;
}

static K_MUTEX_DEFINE(at_write_mutex);
static inline int at_replyn(const char *s, size_t len)
{
	k_mutex_lock(&at_write_mutex, K_FOREVER);

	int ret = at_putn(s, len);
	if (ret == 0)
		ret = at_putn(AT_EOL, sizeof(AT_EOL) - 1);

	k_mutex_unlock(&at_write_mutex);
	return ret;
}

int at_reply(const char *s)
{
	return at_replyn(s, strlen(s));
}

#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 1, 2)))
#endif
int at_replyf(const char *fmt, ...)
{
	char buf[256];

	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (n < 0) {
		return n; // Formatting error
	}

	// Truncate, note that `vsnprintf()` does not count the '\0' character, so
	// make sure there is space for it.
	// TODO: Maybe consider returning an error here instead.
	if (n >= sizeof(buf)) {
		n = sizeof(buf) - 1;
	}

	return at_replyn(buf, n);
}
