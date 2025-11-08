
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <ctype.h>

#include <zephyr/app_version.h>
#include <zephyr/logging/log.h>

#include "at_cmd.h"
#include "cmd_uart.h"

LOG_MODULE_REGISTER(at, LOG_LEVEL_DBG);

static int at_cmd_ID(const struct at_cmd_param *arg, void *ctx)
{
	at_replyf("%s %s PressureSensor=None ForceSensor=None", CONFIG_APP_PROJECT_NAME, APP_VERSION_STRING);
	return 0;
}

static int at_cmd_LA(const struct at_cmd_param *arg, void *ctx)
{
	// HACK: Just return some dummy data for now to make the WebGUI happy
	LOG_WRN("AT LA not implemented, returning dummy data");

	at_reply("Slot:mouse");
	at_reply("AT SC 0x00ffff");
	at_reply("AT SB 2");
	at_reply("AT AX 60");
	at_reply("AT AY 60");
	at_reply("AT DX 20");
	at_reply("AT DY 20");
	at_reply("AT MS 50");
	at_reply("AT AC 50");
	at_reply("AT TS 500");
	at_reply("AT TP 525");
	at_reply("AT SP 700");
	at_reply("AT SS 300");
	at_reply("AT MM 1");
	at_reply("AT RO 0");
	at_reply("AT BT 1");
	at_reply("AT BM 1");
	at_reply("AT KP KEY_UP");
	at_reply("AT BM 2");
	at_reply("AT KP KEY_DOWN");
	at_reply("AT BM 3");
	at_reply("AT KP KEY_LEFT");
	at_reply("AT BM 4");
	at_reply("AT KP KEY_RIGHT");
	at_reply("END");

	return 0;
}

enum mouse_axis {
	MOUSE_AXIS_X,
	MOUSE_AXIS_Y,
};

// HACK: Do not use hog_push_report() directly from here, instead
// have a proper queuing system in place. But for a quick demo
// it will do.
extern void hog_push_report(int8_t btn, int8_t x, int8_t y);
static int at_cmd_Mx(const struct at_cmd_param *arg, void *ctx)
{
	int d;
	enum mouse_axis a = (enum mouse_axis)ctx;

	int ret = at_param_get_int(arg, &d);
	if (ret < 0)
		return ret;

	LOG_WRN("Directly using hog_push_report()");
	if (a == MOUSE_AXIS_X) {
		hog_push_report(0, d, 0);
	} else if (a == MOUSE_AXIS_Y) {
		hog_push_report(0, 0, d);
	}

	return 0;
}

static const struct at_cmd at_cmds[] = {
	{ MAKE2CC(ID), AT_PARAM_NONE, at_cmd_ID, NULL },

	{ MAKE2CC(MX), AT_PARAM_INT, at_cmd_Mx, (void *)MOUSE_AXIS_X},
	{ MAKE2CC(MY), AT_PARAM_INT, at_cmd_Mx, (void *)MOUSE_AXIS_Y},

	{ MAKE2CC(LA), AT_PARAM_NONE, at_cmd_LA, NULL },
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

// TODO: Consider reworking this without a mutex but instead queueing
// up commands. However we would need to make a copy of cmd + param.
// The `at_cmd_param_clone()` will do a heap alloc so maybe we do not
// want that.
// Consider rewriting `at_cmd_param` to make use of string inlining for
// short strings and use a slab allocator for longer strings.
static K_MUTEX_DEFINE(at_dispatch_lock);
static int at_dispatch_internal(const struct at_cmd *cmd, const struct at_cmd_param *param)
{
	char buf[3];

	if (!cmd || !param)
		return -EINVAL;

	if (k_is_in_isr()) {
		LOG_ERR("at_dispatch_internal() called from ISR");
		return -EWOULDBLOCK;
	}

	if (!cmd->cb) {
		// Do not fail here fully but print a warning
		LOG_WRN("Callback for AT command <%s> not implemented", at_code_to_str(cmd->code, buf));
		return 0;
	}


	k_mutex_lock(&at_dispatch_lock, K_FOREVER);

	// TODO: Print/log the parameter too
	LOG_DBG("Dispatching <%s>", at_code_to_str(cmd->code, buf));
	int ret = cmd->cb(param, cmd->ctx);

	k_mutex_unlock(&at_dispatch_lock);

	return ret;
}

int at_dispatch_cmd(const uint16_t code, const struct at_cmd_param *param)
{
	const struct at_cmd *cmd = find_at_cmd(code);
	if (cmd == NULL) {
		char buf[3];
		LOG_ERR("Could not find AT cmd <%s>", at_code_to_str(code, buf));
		return -ENOTSUP;
	}

	return at_dispatch_internal(cmd, param);
}

// Copies the string first to the stack because our parser will modify it
// in place. This is needed when we are parsing string literals.
int at_handle_line_copy(const char *s)
{
	if (!s) {
		return -EINVAL;
	}

	size_t len = strlen(s);
	if (len == 0) {
		return -EINVAL;
	}

	if (len > AT_LINE_MAX) {
		return -EMSGSIZE;
	}

	char buf[AT_LINE_MAX + 1];
	memcpy(buf, s, len + 1);
	return at_handle_line(buf);
}

int at_handle_line_mut(char *s)
{
	if (!s) {
		return -EINVAL;
	}

	// Remove any trailing and leading whitespace
	s = ltrim(s);
	s = rtrim(s);

	if (*s == '\0') {
		return -EINVAL;
	}

	if (strncasecmp(s, "AT", 2) != 0) {
		return -EINVAL;
	}
	s += 2;

	// Plain AT command just return OK
	if (*s == '\0') {
		at_reply("OK");
		return 0;
	}

	// The next character following AT must be at least one space or tab.
	// If this is not the case then there is some error
	if (*s != ' ' && *s != '\t') {
		return -EINVAL;
	}

	// Trim possible multiple whitespace characters after the AT command
	s = ltrim(s);

	// This will be the start of our command
	char *command = s;
	char *param = NULL;

	// Find the end of the command
	while (*s != '\0' && *s != ' ' && *s != '\t') {
		s++;
	}

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
	if (cmd->param_type == AT_PARAM_NONE && param != NULL) {
		LOG_WRN("AT cmd <%s> does not take parameters, ignoring <%s>", command, param);
	}

	// Fail if a parameter is missing
	if (cmd->param_type != AT_PARAM_NONE && param == NULL) {
		LOG_WRN("AT cmd <%s> requires a parameter", command);
		return -EINVAL;
	}

	struct at_cmd_param cmd_param = { .type = cmd->param_type };

	switch (cmd->param_type) {
		case AT_PARAM_NONE:
			break;
		case AT_PARAM_INT:
			cmd_param.val.i = strtol(param, NULL, 0);
			break;
		case AT_PARAM_UINT:
			cmd_param.val.ui = strtoul(param, NULL, 0);
			break;
		case AT_PARAM_STR:
			cmd_param.val.s = param;
			break;
		default:
			LOG_WRN("Unhandled param_type %d", cmd->param_type);
			break;
	};

	return at_dispatch_internal(cmd, &cmd_param);
}

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

static inline int at_replyn(const char *s, size_t len)
{
	int ret = at_putn(s, len);
	if (ret < 0)
		return ret;

	return at_putn(AT_EOL, sizeof(AT_EOL) - 1);
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

struct at_cmd_param *at_cmd_param_clone(const struct at_cmd_param *param)
{
	if (param == NULL)
		return NULL;

	struct at_cmd_param *np = k_malloc(sizeof(*param));
	if (np == NULL)
		return NULL;

	// Copy the type
	np->type = param->type;

	// Copy the value (allocate space for the string parameter if needed)
	switch (np->type) {
		case AT_PARAM_NONE:
			break;
		case AT_PARAM_INT:
			np->val.i = param->val.i;
			break;
		case AT_PARAM_UINT:
			np->val.ui = param->val.ui;
			break;
		case AT_PARAM_STR:
			size_t len = strlen(param->val.s) + 1;
			np->val.s = k_malloc(len);
			if (!np->val.s) {
				k_free(np);
				return NULL;
			}
			memcpy(np->val.s, param->val.s, len);
			break;
		default:
			LOG_WRN("Unknown param type %d", param->type);
			break;
	}

	return np;
}

// TODO: Do not use `k_malloc()`/`k_free()` and instead use slab alloc
void at_cmd_param_free(struct at_cmd_param *param)
{
	if (param == NULL)
		return;

	if (param->type == AT_PARAM_STR)
		k_free(param->val.s);

	k_free(param);
}
