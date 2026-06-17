// SPDX-License-Identifier: GPL-3.0-or-later

#include  <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/retained_mem.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include "app_power_manager.h"
#include "at_cmd.h"
#include "ble.h"
#include "button_manager.h"
#include "cmd_uart.h"
#include "motion_engine.h"
#include "slot_manager.h"
#include "telemetry_uart.h"

// TODO: Define use-cases and behavior for pairing, re-pairing,
// and connection to multiple devices. (General vs. Limited Discoverable)

LOG_MODULE_REGISTER(theadmouse, CONFIG_THEADMOUSE_LOG_LEVEL);

int main(void)
{
	int ret;

	LOG_INF("Tina was here ^_^");

	ret = slot_manager_init();
	if (ret < 0) {
		LOG_ERR("Slot manager initialization failed");
	}

	ret = ble_init();
	if (ret < 0) {
		LOG_ERR("BLE initialization failed");
	}

#ifdef CONFIG_TELEMETRY
	ret = telemetry_uart_init();
	if (ret < 0) {
		LOG_ERR("telemetry uart initialization failed");
	}
#endif

	ret = button_manager_init();
	if (ret < 0) {
		LOG_ERR("button manager initialization fialed");
	}

	ret = cmd_uart_init();
	if (ret < 0) {
		LOG_ERR("cmd uart initialization failed");
	}

	ret = motion_engine_init();
	if (ret < 0) {
		LOG_ERR("motion engine initialization failed");
	}

	ret = app_pm_init();
	if (ret < 0) {
		LOG_ERR("App power-manager initialization failed");
	}

	k_sleep(K_FOREVER);

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

	int ret = at_parse_line_inplace(line, &cmd, &cmd_param, AT_PARSER_FLAG_ALLOW_NO_PREFIX);
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

static int cmd_app_pm_ra_handler(const struct shell *shell, size_t argc, char **argv)
{
	app_pm_report_activity();
	return 0;
}

static int cmd_app_pm_idle_handler(const struct shell *shell, size_t argc, char **argv)
{
	app_pm_debug_execute_idle_work_handler();
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	app_pm_cmds,
	SHELL_CMD(ra, NULL, "Report activity", cmd_app_pm_ra_handler),
	SHELL_CMD(idle, NULL, "Execute idle work handler (simulate idle timeout)", cmd_app_pm_idle_handler),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(app_pm, &app_pm_cmds, "Application power-management commands", NULL);

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

static int cmd_slot_manager_play_tone_hanlder(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2 || argc > 3) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	int freq = CLAMP(atoi(argv[1]), 0, 24000);
	int duration_ms = 1000;
	if (argc == 3)
		duration_ms = CLAMP(atoi(argv[2]), 0, 10000);

	shell_print(shell, "Enqueueing tone playback with %d Hz for %d ms", freq, duration_ms);
	return slot_manager_play_tone(freq, duration_ms);
}

static int cmd_slot_manager_set_rgb_led_handler(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	int r = CLAMP(atoi(argv[1]), 0, 255);
	int g = CLAMP(atoi(argv[2]), 0, 255);
	int b = CLAMP(atoi(argv[3]), 0, 255);

	shell_print(shell, "Setting RGB LED color: %d, %d, %d", r, g, b);

	return slot_manager_set_rgb_led_color(r, g, b);
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	slot_manager_cmds,
	SHELL_CMD_ARG(play_tone, NULL,
			"Play tone on the PWM speaker\n"
			"Usage: play_tone <frequency> [duration]",
			cmd_slot_manager_play_tone_hanlder, 2, 1),
	SHELL_CMD_ARG(set_rgb_led, NULL,
			"Set RGB LED color\n"
			"Usage: set_rgb_led <red> <green> <blue>",
			cmd_slot_manager_set_rgb_led_handler, 4, 0),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(slot_manager, &slot_manager_cmds, "Slot manager commands", NULL);

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
