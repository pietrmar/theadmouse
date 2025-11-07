// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <sys/types.h>

int telemetry_uart_init(void);
int telemetry_uart_printf(const char *fmt, ...);
