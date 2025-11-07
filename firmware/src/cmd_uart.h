// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <sys/types.h>

int cmd_uart_init(void);
ssize_t cmd_uart_write(const uint8_t *data, size_t len);
