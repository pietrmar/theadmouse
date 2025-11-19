// SPDX-License-Identifier: GPL-3.0-or-later

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "cmd_uart.h"
#include "at_cmd.h"

LOG_MODULE_REGISTER(cmd_uart, LOG_LEVEL_DBG);

// TODO: Don't make this a full compile-time failure when the `mpi,cmd-uart`
// property is missing.
static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(mpi_cmd_uart));

#define CMD_RX_LINE_MAX		(AT_LINE_MAX + 1)
#define CMD_RX_LINEQ_DEPTH	4

static char cur_rx_line[CMD_RX_LINE_MAX];
static uint16_t cur_rx_len;
K_MSGQ_DEFINE(line_q, sizeof(cur_rx_line), CMD_RX_LINEQ_DEPTH, 4);

static void line_rx_thread(void *, void *, void *);
K_THREAD_DEFINE(line_rx_tid, 2048, line_rx_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, 0);


#define CMD_TX_BUF_SIZE	4096
#define CMD_TX_CHUNK	256
RING_BUF_DECLARE(cmd_tx_rb, CMD_TX_BUF_SIZE);
static struct k_spinlock cmd_tx_rb_lock;


static void uart_isr(const struct device *dev, void *user_data)
{
	// TODO: For real UARTs (not CDC-ACM) we probably don't want to loop here
	// and instead do a single check.
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[CMD_RX_LINE_MAX];
			int ret;

			do {
				ret = uart_fifo_read(dev, buf, sizeof(buf));

				if (ret < 0) {
					LOG_ERR("Failed to read from FIFO: %d", ret);
					break;
				}

				for (int i = 0; i < ret; i++) {
					// Process the individual characters
					char c = buf[i];

					if (c == '\r' || c == '\n') {
						if (cur_rx_len > 0) {
							cur_rx_line[cur_rx_len] = '\0';

							int qret = k_msgq_put(&line_q, &cur_rx_line, K_NO_WAIT);
							if (qret < 0) {
								LOG_ERR("Failed to enqueue message: %d", qret);
							}

							cur_rx_len = 0;
						}

						continue;
					}

					if (cur_rx_len < (sizeof(cur_rx_line) - 1)) {
						cur_rx_line[cur_rx_len] = c;
						cur_rx_len++;
					} else {
						cur_rx_line[cur_rx_len] = '\0';
						LOG_ERR("Line too long, dropped: %s", cur_rx_line);
						cur_rx_len = 0;
					}
				}

			} while (ret > 0);
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buf[CMD_TX_CHUNK];

			k_spinlock_key_t key = k_spin_lock(&cmd_tx_rb_lock);
			uint32_t rb_len = ring_buf_get(&cmd_tx_rb, buf, sizeof(buf));
			k_spin_unlock(&cmd_tx_rb_lock, key);
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

ssize_t cmd_uart_write(const uint8_t *data, size_t len)
{
	if (!uart_host_is_ready()) {
		return -ENOTCONN;
	}

	size_t written = 0;
	while (written < len) {
		k_spinlock_key_t key = k_spin_lock(&cmd_tx_rb_lock);
		uint32_t rb_written = ring_buf_put(&cmd_tx_rb, &data[written], (uint32_t)(len - written));
		k_spin_unlock(&cmd_tx_rb_lock, key);

		if (rb_written == 0) {
			// TODO: Consider not busy looping and isntead returning the number of bytes
			// writte so far.
			k_sleep(K_TICKS(1));
			continue;
		}

		written += rb_written;
		uart_irq_tx_enable(uart_dev);
	}

	return written;
}

static void line_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	char line[CMD_RX_LINE_MAX];

	while (true) {
		k_msgq_get(&line_q, &line, K_FOREVER);

		LOG_DBG("Rx: %s", line);
		int ret = at_handle_line_inplace(line, 0);
		if (ret < 0) {
			LOG_ERR("AT handling failed: %d (line: <%s>)", ret, line);
		}
	}
}

int cmd_uart_init(void)
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
	uart_irq_rx_enable(uart_dev);

	LOG_INF("uart is ready");

	return 0;
}
