// SPDX-License-Identifier: GPL-3.0-or-later

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "telemetry_uart.h"

LOG_MODULE_REGISTER(telemetry_uart, LOG_LEVEL_INF);

#define TELEMETRY_TX_BUF_SIZE	2048
#define TELEMETRY_TX_CHUNK	256
RING_BUF_DECLARE(telemetry_tx_rb, TELEMETRY_TX_BUF_SIZE);
static struct k_spinlock telemetry_tx_rb_lock;
K_MUTEX_DEFINE(telemetry_tx_call_lock);


// TODO: Don't make this a full compile-time failure when the `mpi,telemetry-uart`
// property is missing.
static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(mpi_telemetry_uart));

static void uart_isr(const struct device *dev, void *user_data)
{
	// TODO: For real UARTs (not CDC-ACM) we probably don't want to loop here
	// and instead do a single check.
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_tx_ready(dev)) {
			uint8_t buf[TELEMETRY_TX_CHUNK];

			k_spinlock_key_t key = k_spin_lock(&telemetry_tx_rb_lock);
			uint32_t rb_len = ring_buf_get(&telemetry_tx_rb, buf, sizeof(buf));
			k_spin_unlock(&telemetry_tx_rb_lock, key);
			if (rb_len == 0) {
				// Ringbuffer empty, disable TX interrupts
				uart_irq_tx_disable(dev);
				continue;
			}

			int send_len = uart_fifo_fill(dev, buf, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Dropped %d bytes", rb_len - send_len);
			}
		}
	}
}

// TODO: For hardware UARTs this should just always return true
static inline bool uart_host_is_ready()
{
	uint32_t dtr = 0;

	int ret = uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
	if (ret < 0) {
		LOG_ERR("Failed to get DTR status");
		return false;
	}

	return (dtr != 0);
}

// NOTE: This cannot be called from interrupt context becasue of the sleep and mutex lock.
static ssize_t telemetry_uart_write(const uint8_t *data, size_t len)
{
	if (!uart_host_is_ready()) {
		return -ENOTCONN;
	}

	k_mutex_lock(&telemetry_tx_call_lock, K_FOREVER);

	size_t written = 0;
	while (written < len) {
		k_spinlock_key_t key = k_spin_lock(&telemetry_tx_rb_lock);
		uint32_t rb_written = ring_buf_put(&telemetry_tx_rb, &data[written], (uint32_t)(len - written));
		k_spin_unlock(&telemetry_tx_rb_lock, key);

		if (rb_written == 0) {
			// TODO: This is for sure problematic to be called from an ISR
			k_sleep(K_TICKS(1));
			continue;
		}

		written += rb_written;
		uart_irq_tx_enable(uart_dev);
	}

	k_mutex_unlock(&telemetry_tx_call_lock);

	return written;
}

int telemetry_uart_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("uart device is not ready");
		return -ENODEV;
	}

	int ret = uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
	if (ret < 0) {
		LOG_ERR("Failed to set uart IRQ callback");
		return ret;
	}

	// Disable TX IRQ until we are sending data
	uart_irq_tx_disable(uart_dev);

	// We are not receiving anything, keep RX IRQ disabled
	uart_irq_rx_disable(uart_dev);

	LOG_INF("uart is ready");

	return 0;
}

int telemetry_uart_printf(const char *fmt, ...)
{
	char buf[256];

	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (n < 0) {
		return n; // Formatting error
	}

	// Truncate, note that `vsnprintf()` does not count the '\0' character, so
	// make sure there is space for it.
	// TODO: Maybe consider returning an error here instead.
	if (n >= sizeof(buf)) {
		n = sizeof(buf) - 1;
	}

	size_t written = telemetry_uart_write(buf, n);
	return written;
}
