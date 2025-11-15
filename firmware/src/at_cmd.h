#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

#include "at_cmd_param.h"

#define AT_LINE_MAX	255
#define AT_EOL		"\r\n"

#define AT_FLAG_PARSER_ALLOW_NO_PREFIX		(1 << 0)

// TODO: Maybe have some param flags like `AT_PARAM_RADIX_HEX`, `AT_PARAM_RADIX_DEC`, etc.


typedef int (*at_cmd_fn)(const struct at_cmd_param *arg, void *ctx);


#define MAKE2CC(x) ((uint16_t)(#x[0]) | ((uint16_t)(#x[1]) << 8))

static inline uint16_t at_code_from_str(const char buf[2])
{
	return ((uint16_t)buf[0]) | ((uint16_t)buf[1] << 8);
}

static inline const char *at_code_to_str(uint16_t code, char buf[3])
{
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

int at_handle_line_inplace(char *s, uint32_t flags);
int at_handle_line_copy(const char *s, uint32_t flags);

int at_parse_line_inplace(char *s, const struct at_cmd **out_cmd, struct at_cmd_param *out_cmd_param, uint32_t flags);
int at_parse_line_copy(const char *s, const struct at_cmd **out_cmd, struct at_cmd_param *out_cmd_param, uint32_t flags);

int at_cmd_enqueue_code(const uint16_t code, const struct at_cmd_param *param, k_timeout_t timeout);
int at_cmd_enqueue_ptr(const struct at_cmd *cmd, const struct at_cmd_param *param, k_timeout_t timeout);

int at_cmd_dispatch_code(const uint16_t code, const struct at_cmd_param *param);
int at_cmd_dispatch_ptr(const struct at_cmd *cmd, const struct at_cmd_param *param);

int at_format_cmd(char *out_buf, size_t buf_len, const uint16_t code, const struct at_cmd_param *param);

int at_reply(const char *s);
int at_replyf(const char *fmt, ...);
