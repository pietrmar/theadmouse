#pragma once

#include "at_cmd.h"

int button_manager_init(void);

int button_manager_set_mapping(int idx, uint16_t at_cmd, const struct at_cmd_param *at_param);
int button_manager_delete_mapping(int idx);
