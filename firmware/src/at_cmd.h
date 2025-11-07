#pragma once

#define AT_LINE_MAX	255
#define AT_EOL		"\r\n"

enum at_param_type {
	AT_PARAM_NONE = 0,
	AT_PARAM_INT,
	AT_PARAM_UINT,
	AT_PARAM_STR,
};

struct at_cmd_param {
	enum at_param_type type;

	union {
		int32_t	i;
		uint32_t ui;
		char *s;
	} val;
};

typedef int (*at_cmd_fn)(const struct at_cmd_param *arg, void *ctx);

struct at_cmd {
	const char *name;
	enum at_param_type param_type;
	at_cmd_fn cb;
	void *ctx;
};

int at_handle_line_copy(const char *s);
int at_handle_line_mut(char *s);

// This automatically selects the copy variant if a string literal is used
#define at_handle_line(s) \
	(__builtin_constant_p(s) ? at_handle_line_copy((s)) : at_handle_line_mut((s)))

int at_dispatch_cmd(const struct at_cmd *cmd, struct at_cmd_param *param);

int at_reply(const char *s);
int at_replyf(const char *fmt, ...);

static inline int at_param_get_int(const struct at_cmd_param *p, int32_t *out)
{
	if (!p || !out) return -ENODATA;
	if (p->type != AT_PARAM_INT) return -EINVAL;
	*out = p->val.i;
	return 0;
}

static inline int at_param_get_uint(const struct at_cmd_param *p, uint32_t *out)
{
	if (!p || !out) return -ENODATA;
	if (p->type != AT_PARAM_UINT) return -EINVAL;
	*out = p->val.ui;
	return 0;
}

static inline int at_param_get_str(const struct at_cmd_param *p, char **out)
{
	if (!p || !out) return -ENODATA;
	if (p->type != AT_PARAM_STR) return -EINVAL;
	*out = p->val.s;
	return 0;
}
