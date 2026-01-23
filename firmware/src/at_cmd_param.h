#pragma once

#include <stdint.h>
#include <string.h>

#define AT_CMD_PARAM_INLINE_STR_SIZE	16

enum at_cmd_param_type {
	AT_CMD_PARAM_TYPE_NONE	= 0,
	AT_CMD_PARAM_TYPE_INT,
	AT_CMD_PARAM_TYPE_UINT,
	AT_CMD_PARAM_TYPE_STR,
};

enum at_cmd_param_flags {
	AT_CMD_PARAM_FLAG_NONE		= 0,
	AT_CMD_PARAM_FLAG_HEAPSTR	= (1 << 0),
};

struct at_cmd_param {
	enum at_cmd_param_type type;
	enum at_cmd_param_flags flags;

	// TODO: Drop the `val` and just access the union members directly
	union {
		int32_t	i;
		uint32_t ui;
		char is[AT_CMD_PARAM_INLINE_STR_SIZE];
		char *hs;
	} val;
};

static inline enum at_cmd_param_type at_cmd_param_get_type(const struct at_cmd_param *p)
{
	return p->type;
}

static inline void at_cmd_param_set_type(struct at_cmd_param *p, enum at_cmd_param_type type)
{
	// TODO: If it was a heap-string this is probably the place to free it here before setting
	// a new type, maybe?
	p->type = type;
}

int at_cmd_param_clone(struct at_cmd_param *dst, const struct at_cmd_param *src, struct k_heap *heap);
void at_cmd_param_free(struct at_cmd_param *p, struct k_heap *heap);

static inline int at_cmd_param_get_int(const struct at_cmd_param *p, int32_t *out)
{
	if (!p || !out) return -EINVAL;
	if (at_cmd_param_get_type(p) != AT_CMD_PARAM_TYPE_INT) return -EINVAL;
	*out = p->val.i;
	return 0;
}

static inline int at_cmd_param_set_int(struct at_cmd_param *p, int32_t val)
{
	if (!p) return -EINVAL;
	at_cmd_param_set_type(p, AT_CMD_PARAM_TYPE_INT);
	p->val.i = val;
	return 0;
}

static inline int at_cmd_param_get_uint(const struct at_cmd_param *p, uint32_t *out)
{
	if (!p || !out) return -EINVAL;
	if (at_cmd_param_get_type(p) != AT_CMD_PARAM_TYPE_UINT) return -EINVAL;
	*out = p->val.ui;
	return 0;
}

static inline int at_cmd_param_set_uint(struct at_cmd_param *p, uint32_t val)
{
	if (!p) return -EINVAL;
	at_cmd_param_set_type(p, AT_CMD_PARAM_TYPE_UINT);
	p->val.ui = val;
	return 0;
}

static inline int at_cmd_param_get_str(const struct at_cmd_param *p, const char **out)
{
	if (!p || !out) return -EINVAL;
	if (at_cmd_param_get_type(p) != AT_CMD_PARAM_TYPE_STR) return -EINVAL;

	if (p->flags & AT_CMD_PARAM_FLAG_HEAPSTR)
		*out = p->val.hs;
	else
		*out = p->val.is;

	return 0;
}

// TODO: Figure out how to handle heap string allocations and when to allocate and free
// them.
static inline int at_cmd_param_set_str(struct at_cmd_param *p, const char *val)
{
	if (!p || !val) return -EINVAL;

	size_t len = strlen(val);
	if (len >= AT_CMD_PARAM_INLINE_STR_SIZE) {
		return -EOVERFLOW;
	}
	at_cmd_param_set_type(p, AT_CMD_PARAM_TYPE_STR);
	memcpy(p->val.is, val, len + 1);
	return 0;
}
