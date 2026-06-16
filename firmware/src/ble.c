#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/drivers/hwinfo.h>

#include <zephyr/settings/settings.h>

#include <zephyr/logging/log.h>

#include "ble_hid_service.h"

LOG_MODULE_REGISTER(ble, CONFIG_THEADMOUSE_LOG_LEVEL);

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
static int ble_start_adv(void)
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

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	LOG_INF("Updated MTU is: %u", bt_gatt_get_mtu(conn));
}

static struct bt_gatt_exchange_params mtu_exchange_params = {
	.func = mtu_exchange_cb,
};

// TODO: Check if we need to flush any HID queues or so when a device (re-)connected.
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

// TODO: After the device was in suspend for a while it seems that my notebook does
// not automatically reconnect to it.
// TODO: Consider instead lowering the connection interval and such to a minimum.
int ble_suspend(void)
{
	LOG_INF("Suspending");

	int ret = bt_disable();
	if (ret < 0) {
		LOG_ERR("Failed to disable BT: %d", ret);
		return ret;
	}

	return 0;
}

int ble_resume(void)
{
	LOG_INF("Resuming");
	return ble_init();
}
