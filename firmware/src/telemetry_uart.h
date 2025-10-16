#pragma once

int telemetry_uart_init(void);
int telemetry_uart_printf(const char *fmt, ...);

size_t telemetry_uart_write(const uint8_t *data, size_t len);
