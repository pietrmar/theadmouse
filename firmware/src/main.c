#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/retained_mem.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include "hog.h"
#include "MadgwickAHRS/MadgwickAHRS.h"

LOG_MODULE_REGISTER(theadmouse, CONFIG_THEADMOUSE_LOG_LEVEL);

static const struct bt_data le_adv[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
			BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
			BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data le_scan_rsp[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

int ble_start_adv(void)
{
	return bt_le_adv_start(BT_LE_ADV_CONN, le_adv, ARRAY_SIZE(le_adv), le_scan_rsp, ARRAY_SIZE(le_scan_rsp));
}

int ble_init(void)
{
	int ret;

	ret = bt_enable(NULL);
	if (ret < 0) {
		LOG_ERR("Failed to enable BT: %d", ret);
		return ret;
	}

	ret = settings_load_subtree("bt");
	if (ret < 0) {
		LOG_ERR("Failed to load 'bt' settings subtree");
		return ret;
	}

	ret = hog_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize HOG: %d", ret);
		return ret;
	}

	ret = ble_start_adv();
	if (ret < 0) {
		LOG_ERR("Failed to start BLE advertisement: %d", ret);
		return ret;
	}

	LOG_INF("Started BLE advertisement, device name: %s", CONFIG_BT_DEVICE_NAME);

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

	LOG_INF("MTU is: %u", bt_gatt_get_mtu(conn));
	int ret = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
	if (ret < 0) {
		LOG_ERR("Failed to increase MTU: %d", ret);
	}
	LOG_INF("Connected to %s", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected from %s, reason: %s (%#04x)", addr, bt_hci_err_to_str(reason), reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static bool first_measurement = true;
static uint64_t last_time = 0;
static uint32_t trigger_cnt = 0;

static void lsm6dsl_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];

	trigger_cnt++;

	sensor_sample_fetch(dev);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

	// TODO: Make sure this works correctly with overflows and such
	uint32_t current_time = k_cyc_to_us_near64(k_cycle_get_32());
	double dt = (current_time - last_time) / 1000000.0;
	last_time = current_time;


	// If this is the first measurement then just do nothing, we just
	// use it to have a valid `last_time` value.
	if (first_measurement) {
		first_measurement = false;
		return;
	}

	float ax = sensor_value_to_float(&accel[0]) / 9.81f;
	float ay = sensor_value_to_float(&accel[1]) / 9.81f;
	float az = sensor_value_to_float(&accel[2]) / 9.81f;

	float gx = sensor_value_to_float(&gyro[0]);
	float gy = sensor_value_to_float(&gyro[1]);
	float gz = sensor_value_to_float(&gyro[2]);

	sampleFreq = 1.0f / dt;
	MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);

	if ((trigger_cnt % 5) == 0) {
		// w, x, y, z
		float q[4] = { q0, q1, q2, q3 };
		int16_t q_fixed[4];
		for (int i = 0; i < 4; i++) {
			q_fixed[i] = (int16_t)(q[i] * 10000);  // scale to fit -1.0 to 1.0 as -10000 to 10000
		}

		uint8_t payload[8];
		payload[0] = q_fixed[0] & 0xFF;
		payload[1] = q_fixed[0] >> 8;
		payload[2] = q_fixed[1] & 0xFF;
		payload[3] = q_fixed[1] >> 8;
		payload[4] = q_fixed[2] & 0xFF;
		payload[5] = q_fixed[2] >> 8;
		payload[6] = q_fixed[3] & 0xFF;
		payload[7] = q_fixed[3] >> 8;
		bt_nus_send(NULL, &payload, sizeof(payload));
	}
}

static int lsm6dsl_init()
{
	int ret;
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	if (!device_is_ready(lsm6dsl_dev)) {
		LOG_ERR("lsm6dsl is  not ready");
		return -ENODEV;
	}

	// Possible values from the data-sheet:
	//  1.6, 12.5, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664
	struct sensor_value odr_value = { 104, 0 };

	ret = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_value);
	if (ret < 0) {
		LOG_ERR("Failed to set accelerometer sampling frequency: %d", ret);
		return ret;
	}

	ret = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_value);
	if (ret < 0) {
		LOG_ERR("Failed to set gyroscope sampling frequency: %d", ret);
		return ret;
	}


	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	ret = sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler);
	if (ret < 0) {
		LOG_ERR("Failed to set lsm6dsl trigger: %d", ret);
		return ret;
	}

	return 0;
}

int main(void)
{
	int ret;

	LOG_INF("Tina was here ^_^");

	ret = ble_init();
	if (ret < 0) {
		LOG_ERR("BLE initialization failed");
	}

	ret = lsm6dsl_init();
	if (ret < 0) {
		LOG_ERR("lsm6dsl initialization failed");
	}

	while (true) {
#if 0
		hog_push_report(0, 100, 0);
		k_msleep(1000);
		hog_push_report(0, -100, 0);
		k_msleep(1000);
#else
		k_msleep(1000);
#endif
	}

	return 0;
}

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
SHELL_CMD_REGISTER(wipenvs, NULL, "Wipe the NVS settings partition adn reboot", wipenvs_handler);
