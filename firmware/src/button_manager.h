#pragma once

#include "at_cmd.h"

int button_manager_init(void);

int button_manager_set_mapping(size_t idx, uint16_t at_cmd, const struct at_cmd_param *at_param);
int button_manager_delete_mapping(size_t idx);


extern const size_t button_manager_num_buttons;

static inline bool button_manager_valid_index(size_t idx)
{
	return idx < button_manager_num_buttons;
}
