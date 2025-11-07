
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

#define AT_EOL "\r\n"

int at_cmd_ID(const struct at_cmd_param *arg, void *ctx)
{
	at_replyf("%s %s PressureSensor=None ForceSensor=None", CONFIG_APP_PROJECT_NAME, APP_VERSION_STRING);
	return 0;
}

int at_cmd_LA(const struct at_cmd_param *arg, void *ctx)
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
int at_cmd_Mx(const struct at_cmd_param *arg, void *ctx)
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
	{ "ID", AT_PARAM_NONE, at_cmd_ID, NULL },

	{ "MX", AT_PARAM_INT, at_cmd_Mx, (void *)MOUSE_AXIS_X},
	{ "MY", AT_PARAM_INT, at_cmd_Mx, (void *)MOUSE_AXIS_Y},

	{ "LA", AT_PARAM_NONE, at_cmd_LA, NULL },
};

static const struct at_cmd *find_at_cmd(const char *cmd)
{
	for (int i = 0; i < ARRAY_SIZE(at_cmds); i++) {
		if (strcasecmp(cmd, at_cmds[i].name) == 0) {
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

int at_dispatch_cmd(const struct at_cmd *cmd, struct at_cmd_param *param)
{
	if (!cmd || !param)
		return -EINVAL;

	if (!cmd->cb) {
		// Do not fail here fully but print a warning
		LOG_WRN("Callback for AT command <%s> not implemented", cmd->name);
		return 0;
	}

	// TODO: Print/log the parameter too
	LOG_DBG("Dispatching <%s>", cmd->name);

	// TODO: Consider serializing this with a mutex in case individual
	// locking inside the callbacks might get too complicated.
	return cmd->cb(param, cmd->ctx);
}

int at_handle_line(char *s, uint16_t len)
{
	if (!s) {
		return -EINVAL;
	}

	// NOTE: The string processing here does not make use of  `len` and adjusts
	// the pointer and string in place, so `len` is not the string length anymore.

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

	// Find the command
	const struct at_cmd *cmd = find_at_cmd(command);
	if (cmd == NULL) {
		LOG_ERR("Could not find AT cmd <%s>", command);
		return -ENOTSUP;
	}

	// Check and print a warning but do not fail if we have extra parameters
	if (cmd->param_type == AT_PARAM_NONE && param != 0) {
		LOG_WRN("AT cmd <%s> does not take parameters, ignoring <%s>", command, param);
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

	return at_dispatch_cmd(cmd, &cmd_param);
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
