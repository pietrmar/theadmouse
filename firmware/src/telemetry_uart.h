// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <sys/types.h>

int telemetry_uart_init(void);
ssize_t telemetry_uart_write(const uint8_t *data, size_t len);
int telemetry_uart_printf(const char *fmt, ...);
