#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

#include "at_cmd_param.h"

#define AT_LINE_MAX	255
#define AT_EOL		"\r\n"

#define AT_FLAG_PARSER_ALLOW_NO_PREFIX		(1 << 0)

enum at_cmd_internal {
	AT_CMD_INTERNAL_BIT		= 0x8000u,
	AT_CMD_INTERNAL_LOAD_SLOT_INDEX	= AT_CMD_INTERNAL_BIT | 0,
};

static const char * const at_cmd_internal_names[] = {
	[0] = "@LOAD_SLOT_INDEX",
};

// TODO: Maybe have some param flags like `AT_PARAM_RADIX_HEX`, `AT_PARAM_RADIX_DEC`, etc.

typedef int (*at_cmd_fn)(const struct at_cmd_param *arg, void *ctx);


#define MAKE2CC(x) ((uint16_t)(#x[0]) | ((uint16_t)(#x[1]) << 8))

static inline uint16_t at_code_from_str(const char buf[2])
{
	return ((uint16_t)buf[0]) | ((uint16_t)buf[1] << 8);
}

static inline const char *at_code_to_str(uint16_t code, char buf[3])
{
	if (code & AT_CMD_INTERNAL_BIT) {
		size_t idx = (code & ~AT_CMD_INTERNAL_BIT);
		if (idx < ARRAY_SIZE(at_cmd_internal_names) && at_cmd_internal_names[idx] != NULL) {
			return at_cmd_internal_names[idx];
		}
		return "@INTERNAL";
	}

	buf[0] = code & 0xFF;
	buf[1] = code >> 8;
	buf[2] = '\0';
	return buf;
}

struct at_cmd {
	uint16_t code;
	enum at_cmd_param_type param_type;
	at_cmd_fn cb;
	void *ctx;
};

// TODO: Remove this API, callers should just manually run parse + enqueue
int at_handle_line_inplace(char *s, uint32_t flags);
int at_handle_line_copy(const char *s, uint32_t flags);

int at_parse_line_inplace(char *s, const struct at_cmd **out_cmd, struct at_cmd_param *out_cmd_param, uint32_t flags);
int at_parse_line_copy(const char *s, const struct at_cmd **out_cmd, struct at_cmd_param *out_cmd_param, uint32_t flags);

// TODO: Remove the `at_cmd_*_code()` variant of the API

// TODO: Add `at_cmd_enqueue_and_wait_*()` variants that wait for the result of the processed command
int at_cmd_enqueue_code(const uint16_t code, const struct at_cmd_param *param, k_timeout_t timeout);
int at_cmd_enqueue_ptr(const struct at_cmd *cmd, const struct at_cmd_param *param, k_timeout_t timeout);

int at_cmd_dispatch_code(const uint16_t code, const struct at_cmd_param *param);
int at_cmd_dispatch_ptr(const struct at_cmd *cmd, const struct at_cmd_param *param);

int at_format_cmd(char *out_buf, size_t buf_len, const uint16_t code, const struct at_cmd_param *param);

int at_reply(const char *s);
int at_replyf(const char *fmt, ...);
