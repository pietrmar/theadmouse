// SPDX-License-Identifier: GPL-3.0-or-later

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/retained_mem.h>

#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include "at_cmd.h"
#include "ble_hid_service.h"
#include "button_manager.h"
#include "cmd_uart.h"
#include "slot_manager.h"
#include "telemetry_uart.h"

// TODO: Define use-cases and behavior for pairing, re-pairing,
// and connection to multiple devices. (General vs. Limited Discoverable)

LOG_MODULE_REGISTER(theadmouse, CONFIG_THEADMOUSE_LOG_LEVEL);

static const struct bt_data le_adv[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
			BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
			BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

// Not `const` because we want to change the name before starting the advertisement
static struct bt_data le_scan_rsp[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

// TODO: We should prevent multiple client connections at the same time, or be able to manage multiple
// simultaneous clients.
int ble_start_adv(void)
{
	le_scan_rsp[0].data = bt_get_name();
	le_scan_rsp[0].data_len = strlen(bt_get_name());

	return bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, le_adv, ARRAY_SIZE(le_adv), le_scan_rsp, ARRAY_SIZE(le_scan_rsp));
}


static int bt_init_friendly_name(void)
{
	uint8_t device_id[8];
	ssize_t ret = hwinfo_get_device_id(device_id, ARRAY_SIZE(device_id));

	if (ret != ARRAY_SIZE(device_id)) {
		LOG_ERR("Failed to get unique device ID: %d", ret);
		return -EIO;
	}

	char name[CONFIG_BT_DEVICE_NAME_MAX + 1];
	snprintf(name, sizeof(name), "%s-%02x%02x", CONFIG_BT_DEVICE_NAME, device_id[ret-2], device_id[ret-1]);

	int res = bt_set_name(name);
	if (res < 0) {
		LOG_ERR("Failed to set BT name: %d", res);
		return res;
	}

	LOG_INF("Device name: %s", bt_get_name());

	return 0;
}

int ble_init(void)
{
	int ret;

	ret = ble_hid_service_init();
	if (ret < 0) {
		LOG_ERR("Failed to start BLE HID service: %d", ret);
		return ret;
	}

	ret = bt_enable(NULL);
	if (ret < 0) {
		LOG_ERR("Failed to enable BT: %d", ret);
		return ret;
	}

	ret = settings_load_subtree("bt");
	if (ret < 0) {
		LOG_ERR("Failed to load 'bt' settings subtree: %d", ret);
		return ret;
	}

	ret = bt_init_friendly_name();
	if (ret < 0) {
		LOG_ERR("Failed to set friendly BT name: %d", ret);
	}

	ret = ble_start_adv();
	if (ret < 0) {
		LOG_ERR("Failed to start BLE advertisement: %d", ret);
		return ret;
	}

	LOG_INF("Started BLE advertisement, device name: %s", bt_get_name());

	return 0;
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	LOG_INF("Updated MTU is: %u", bt_gatt_get_mtu(conn));
}

static struct bt_gatt_exchange_params mtu_exchange_params = {
	.func = mtu_exchange_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Failed to connect to %s, err: %s (%#04x)", addr, bt_hci_err_to_str(err), err);
		return;
	}

	LOG_INF("Connected to %s", addr);

	LOG_INF("MTU is: %u", bt_gatt_get_mtu(conn));
	int ret = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
	if (ret < 0) {
		LOG_ERR("Failed to increase MTU: %d", ret);
	}

	// TODO: This configures the connection interval to 7.5 ms which
	// works nicely with a 100 Hz update rate, however we should make
	// this somehow more connfigurable
	ret = bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(6, 6, 0, 400));
	if (ret < 0) {
		LOG_ERR("Failed do adjust connection interval: %d", ret);
	}

	ret = ble_hid_service_connected(conn);
	if (ret < 0) {
		LOG_ERR("Failed to notify HID service about connection: %d", ret);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected from %s, reason: %s (%#04x)", addr, bt_hci_err_to_str(reason), reason);

	int ret = ble_hid_service_disconnected(conn);
	if (ret < 0)
		LOG_ERR("Failed to notify HID service about disconnection: %d", ret);

	ret = ble_start_adv();
	if (ret < 0)
		LOG_ERR("Failed to start BLE advertisement: %d", ret);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int led_set_rgb(int r, int g, int b)
{
	static const struct led_dt_spec red = LED_DT_SPEC_GET(DT_ALIAS(red_pwm_led));
	static const struct led_dt_spec green = LED_DT_SPEC_GET(DT_ALIAS(green_pwm_led));
	static const struct led_dt_spec blue = LED_DT_SPEC_GET(DT_ALIAS(blue_pwm_led));

	// TODO: Maybe don't error out, instead try best effort to set the LED colors
	// even if some LEDs are not available.
	if (!device_is_ready(red.dev)) {
		LOG_ERR("Red LED device not ready");
		return -ENODEV;
	}
	if (!device_is_ready(green.dev)) {
		LOG_ERR("Green LED device not ready");
		return -ENODEV;
	}
	if (!device_is_ready(blue.dev)) {
		LOG_ERR("Blue LED device not ready");
		return -ENODEV;
	}

	int ret;
	ret = led_set_brightness_dt(&red, r);
	if (ret) {
		LOG_ERR("Seting red LED failed (%d)", ret);
		return ret;
	}

	ret = led_set_brightness_dt(&green, g);
	if (ret) {
		LOG_ERR("Seting green LED failed (%d)", ret);
		return ret;
	}

	ret = led_set_brightness_dt(&blue, b);
	if (ret) {
		LOG_ERR("Seting blue LED failed (%d)", ret);
		return ret;
	}

	return 0;
}

int main(void)
{
	int ret;

	// NOTE: It takes quite a while to arrive here, so consider
	// turning on the green LED via some very early GPIO using
	// `SYS_INIT()` with something like `PRE_KERNEL_1`.
	ret = led_set_rgb(0, 10, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set RGB LED");
	}

	LOG_INF("Tina was here ^_^");

	ret = slot_manager_init();
	if (ret < 0) {
		LOG_ERR("Slot manager initialization failed");
	}

	ret = ble_init();
	if (ret < 0) {
		LOG_ERR("BLE initialization failed");
	}

	ret = telemetry_uart_init();
	if (ret < 0) {
		LOG_ERR("telemetry uart initialization failed");
	}

	ret = button_manager_init();
	if (ret < 0) {
		LOG_ERR("button manager initialization fialed");
	}

	ret = cmd_uart_init();
	if (ret < 0) {
		LOG_ERR("cmd uart initialization failed");
	}

	while (true) {
		k_msleep(1000);
	}

	return 0;
}

#if defined(CONFIG_SHELL)
static int cmd_at(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(shell, "Usage: AT <command_line>");
		return -EINVAL;
	}

	char line[AT_LINE_MAX + 1] = {0};
	size_t offset = 0;

	for (size_t i = 1; i < argc; i++) {
		size_t len = strlen(argv[i]);

		if ((offset + len) >= (sizeof(line) - 1)) {
			shell_print(shell, "Error: command too long");
			return -EMSGSIZE;
		}

		memcpy(line + offset, argv[i], len);
		offset += len;

		if ((i + 1) < argc)
			line[offset++] = ' ';
	}

	line[offset] = '\0';

	const struct at_cmd *cmd = NULL;
	struct at_cmd_param cmd_param = { 0 };

	int ret = at_parse_line_inplace(line, &cmd, &cmd_param, AT_FLAG_PARSER_ALLOW_NO_PREFIX);
	if (ret < 0) {
		shell_error(shell, "Failed to parse line: %d", ret);
		return ret;
	}

	ret = at_cmd_enqueue_and_wait_ptr(cmd, &cmd_param, K_FOREVER);
	if (ret < 0) {
		shell_error(shell, "AT command failed: %d", ret);
		return ret;
	}

	return 0;
}
SHELL_CMD_REGISTER(AT, NULL, "Parse and execute AT command", cmd_at);

static int reboot_handler(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "Rebooting");
	k_msleep(100);
	sys_reboot(SYS_REBOOT_COLD);
}
SHELL_CMD_REGISTER(reboot, NULL, "Perform a reboot", reboot_handler);

static int dfureboot_handler(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *gpregret1 = DEVICE_DT_GET(DT_NODELABEL(gpregret1));

	if (!device_is_ready(gpregret1)) {
		shell_error(shell, "Device is not ready");
		return -ENODEV;
	}

	#define DFU_MAGIC_UF2_RESET 0x57
	uint8_t val = DFU_MAGIC_UF2_RESET;
	int ret = retained_mem_write(gpregret1, 0, &val, sizeof(val));
	if (ret < 0) {
		shell_error(shell, "Failed to write gpregret1: %d", ret);
		return ret;
	}

	shell_print(shell, "Rebooting into DFU mode");
	k_msleep(100);
	sys_reboot(SYS_REBOOT_COLD);

	return 0;
}
SHELL_CMD_REGISTER(dfureboot, NULL, "Reboot into DFU mode (UF2 and CDC)", dfureboot_handler);

#if 0
// TODO: We should not erase the whole flash now as we have LittleFS running on
// it. So think about what to erase exactly.
static int wipenvs_handler(const struct shell *shell, size_t argc, char **argv)
{
	const struct flash_area *fa;

	int ret = flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa);
	if (ret < 0) {
		shell_error(shell, "Failed open settings flash area: %d", ret);
		return ret;
	}

	ret = flash_area_erase(fa, 0, fa->fa_size);
	if (ret < 0) {
		shell_error(shell, "Failed to erase settings flash area: %d", ret);
		return ret;
	}

	flash_area_close(fa);

	shell_print(shell, "Erased settings flash area, rebooting");
	k_msleep(100);
	sys_reboot(SYS_REBOOT_COLD);
}
SHELL_CMD_REGISTER(wipenvs, NULL, "Wipe the NVS settings partition and reboot", wipenvs_handler);
#endif
#endif
