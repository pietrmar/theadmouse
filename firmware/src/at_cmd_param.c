#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "at_cmd_param.h"

LOG_MODULE_DECLARE(at, LOG_LEVEL_DBG);

struct at_cmd_param *at_cmd_param_clone(const struct at_cmd_param *p)
{
	if (p == NULL)
		return NULL;

	if (p->flags & AT_CMD_PARAM_FLAG_HEAPSTR) {
		LOG_ERR("TODO: Implement `at_cmd_param_clone()` for heapstrings");
		return NULL;
	}

	// TODO: Maybe do not use a heap allocation here
	struct at_cmd_param *np = k_malloc(sizeof(*p));
	if (np == NULL)
		return NULL;

	memcpy(np, p, sizeof(*p));
	return np;
}

void at_cmd_param_free(struct at_cmd_param *p)
{
	if (p == NULL)
		return;

	if (p->flags & AT_CMD_PARAM_FLAG_HEAPSTR) {
		// TODO: Free the heapstring
		LOG_ERR("TODO: Implement `at_cmd_param_free()` for heapstring");
	}

	k_free(p);
}
