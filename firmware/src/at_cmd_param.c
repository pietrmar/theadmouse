#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "at_cmd_param.h"

LOG_MODULE_DECLARE(at, LOG_LEVEL_DBG);

int at_cmd_param_clone(struct at_cmd_param *dst, const struct at_cmd_param *src, struct k_heap *heap)
{
	*dst = *src;

	if (src->type == AT_CMD_PARAM_TYPE_STR && src->flags & AT_CMD_PARAM_FLAG_HEAPSTR) {
		size_t len = strlen(src->val.hs);

		if (!heap)
			return -EINVAL;

		char *p = k_heap_alloc(heap, len + 1, K_NO_WAIT);
		if (!p)
			return -ENOMEM;

		strcpy(p, src->val.hs);
		dst->val.hs = p;
	}

	return 0;
}

void at_cmd_param_free(struct at_cmd_param *p, struct k_heap *heap)
{
	if (p->type == AT_CMD_PARAM_TYPE_STR && p->flags & AT_CMD_PARAM_FLAG_HEAPSTR) {
		if (p->val.hs)
			k_heap_free(heap, p->val.hs);
	}
	*p = (struct at_cmd_param){ 0 };
}
