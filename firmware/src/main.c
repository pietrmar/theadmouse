#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/retained_mem.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/spinlock.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include <math.h>

// NOTE: Puck.js MAC: E7:8E:61:16:08:2B
static const char *target_addr_str = "E7:8E:61:16:08:2B";

#if 0
#include "telemetry_uart.h"
#endif

#include "hog.h"
#include "MadgwickAHRS/MadgwickAHRS.h"

// #define THEADMOUSE_NUS_DEBUG

LOG_MODULE_REGISTER(theadmouse, CONFIG_THEADMOUSE_LOG_LEVEL);

#define MFG_ID 0xFFFF

struct __attribute__((packed)) theadmouse_beacon {
	uint16_t mfg_id;
	uint8_t seq;
	uint8_t buttons;
	int8_t dx;
	int8_t dy;
};

#if defined(CONFIG_BOARD_THEADMOUSE)
static const struct bt_data le_adv[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
			BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
			BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),

#if defined(THEADMOUSE_NUS_DEBUG)
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
#endif
};

// Not `const` because we want to change the name before starting the advertisement
static struct bt_data le_scan_rsp[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

int ble_start_adv(void)
{
	le_scan_rsp[0].data = bt_get_name();
	le_scan_rsp[0].data_len = strlen(bt_get_name());

	return bt_le_adv_start(BT_LE_ADV_CONN, le_adv, ARRAY_SIZE(le_adv), le_scan_rsp, ARRAY_SIZE(le_scan_rsp));
}
#elif defined(CONFIG_BOARD_PUCKJS)

#define PUCKJS_USE_PERIODIC_ADV

#if defined(PUCKJS_USE_PERIODIC_ADV)
static struct bt_le_ext_adv *per_le_adv;

static struct bt_data per_default_adv[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

#endif

static struct theadmouse_beacon cur_beacon;

static const struct bt_data le_adv[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	{
		.type = BT_DATA_MANUFACTURER_DATA,
		.data = &cur_beacon,
		.data_len = sizeof(cur_beacon)
	},
};

// Not `const` because we want to change the name before starting the advertisement
static struct bt_data le_scan_rsp[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static struct bt_le_adv_param adv_param = {
	.id           = BT_ID_DEFAULT,
	.sid          = 0,
	.options      = BT_LE_ADV_OPT_USE_IDENTITY,
	.interval_min = 120, // 120 = 75 ms
	.interval_max = 120,
	.peer         = NULL,
};

int ble_start_adv(void)
{
	cur_beacon.mfg_id = MFG_ID;
	cur_beacon.seq = 0;
	cur_beacon.buttons = 0;
	cur_beacon.dx = 0;
	cur_beacon.dy = 0;

#if defined(PUCKJS_USE_PERIODIC_ADV)
	per_default_adv[0].data = bt_get_name();
	per_default_adv[0].data_len = strlen(bt_get_name());

	int ret = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_IDENTITY, NULL, &per_le_adv);
	if (ret < 0) {
		LOG_ERR("Failed to create advertising set: %d", ret);
		return ret;
	}

	ret = bt_le_ext_adv_set_data(per_le_adv, per_default_adv, ARRAY_SIZE(per_default_adv), NULL, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set advertising data: %d", ret);
		return ret;
	}

	// 24 * 1.25 = 30 ms
	// 8 * 1.25 = 10 ms
	ret = bt_le_per_adv_set_param(per_le_adv, BT_LE_PER_ADV_PARAM(8, 8, BT_LE_PER_ADV_OPT_NONE));
	if (ret < 0) {
		LOG_ERR("Failed to set periodic advertising parameters: %d", ret);
		return ret;
	}

	ret = bt_le_per_adv_start(per_le_adv);
	if (ret < 0) {
		LOG_ERR("Failed to enable periodic advertising: %d", ret);
		return ret;
	}


	ret = bt_le_ext_adv_start(per_le_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to start extended advertising: %d", ret);
		return ret;
	}

	ret = bt_le_per_adv_set_data(per_le_adv, le_adv, ARRAY_SIZE(le_adv));
	if (ret < 0) {
		LOG_ERR("Failed to set extended advertising data: %d", ret);
		return ret;
	}
#else
	// Advertising interval_min: 48 = 30 ms
	return bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, 48, 48, NULL), le_adv, ARRAY_SIZE(le_adv), NULL, 0);
#endif
}
#endif

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

#if defined(CONFIG_BOARD_THEADMOUSE)
	ret = hog_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize HOG: %d", ret);
		return ret;
	}
#endif

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

#if defined(CONFIG_BOARD_THEADMOUSE)
static bt_addr_le_t target_addr;
static uint8_t last_seq = 0xFF;

static bool ad_parse_mfg_cb(struct bt_data *data, void *user_data)
{
	const bt_addr_le_t *addr = (const bt_addr_le_t *)user_data;

	if (data->type != BT_DATA_MANUFACTURER_DATA) {
		return true;
	}

	if (data->data_len != sizeof(struct theadmouse_beacon)) {
		return true;
	}

	struct theadmouse_beacon *cur_beacon = data->data;

	if (cur_beacon->mfg_id != MFG_ID) {
		return true;
	}

	if (cur_beacon->seq == last_seq) {
		return false;
	}

	// LOG_INF("%#04x: %d,%d", cur_beacon->seq, cur_beacon->dx, cur_beacon->dy);
	hog_push_report(0, cur_beacon->dx, cur_beacon->dy);

	last_seq = cur_beacon->seq;

	return false;
}

static void scan_recv_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
#if 0
	LOG_INF("ADV %02X:%02X:%02X:%02X:%02X:%02X (%s) RSSI=%d type=0x%02x len=%u",
	addr->a.val[5], addr->a.val[4], addr->a.val[3],
	addr->a.val[2], addr->a.val[1], addr->a.val[0],
	addr->type == BT_ADDR_LE_RANDOM ? "random" : "public",
	rssi, type, ad->len);
#endif

	// TODO: Limit to non-connectable legacy advertisements and drop everything else before
	// even comparing the MAC address.


	// static const char *target_addr_str = "E7:8E:61:16:08:2B";
	// HACK: Something is not working when using `bt_addr_le_cmp()` so let's hardcode it here.
	if (addr->a.val[5] == 0xE7 && addr->a.val[4] == 0x8E, addr->a.val[3] == 0x61,
			addr->a.val[2] == 0x16, addr->a.val[1] == 0x08, addr->a.val[0] == 0x2B) {

		bt_data_parse(ad, ad_parse_mfg_cb, (void *)addr);
	}

	return;
}
#endif

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


#if defined(CONFIG_BOARD_THEADMOUSE)
	static const struct bt_le_scan_param scan_param = {
		.type     = BT_LE_SCAN_TYPE_PASSIVE,
		.options  = BT_LE_SCAN_OPT_NONE,
		.interval = 160, // 100 ms
		.window   = 160,
		.timeout  = 0,
	};


	ret = bt_addr_le_from_str(target_addr_str, "public", &target_addr);
	if (ret < 0) {
		ret = bt_addr_le_from_str(target_addr_str, NULL, &target_addr);
	}
	if (ret < 0) {
		LOG_ERR("Failed to parse MAC address: %d", ret);
	}

	LOG_INF("Starting scan");
	ret = bt_le_scan_start(&scan_param, &scan_recv_cb);
	if (ret < 0) {
		LOG_ERR("Failed to start scan: %d", ret);
	}
#endif
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

#define SENSITIVITY_YAW		800.0f
#define SENSITIVITY_PITCH	800.0f

// Possible values from the data-sheet:
//  1.6, 12.5, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664
// NOTE: From experimentation, the fetching of the samples via
// I2C and calulating the Madgwick filter takes roughly
// ~800-1000 us, so 833 Hz is the sensible maximum here.
// NOTE: Running below 416 Hz increases the gyro drift
#define LSM6DSL_SAMPLE_RATE	416.0f

// Posible values from the data-sheet in g:
//  2, 4, 8, 16
#define LSM6DSL_ACC_FULL_SCALE	16

// Possible values from the data-sheet deg/s:
//  125, 250, 500, 1000, 2000
#define LSM6DSL_GYR_FULL_SCALE	2000

// These will be set in lsm6dsl_init();
static float acc_clip_limit = 0.0f;	// in ms2
static float gyr_clip_limit = 0.0f;	// in rad/s


struct imu_quat {
	float q[4];
};

// TODO: Figure if we actually need a spinlock
static struct k_spinlock quat_lock;
static struct imu_quat last_quat = { .q = { 1.0f, 0.0f, 0.0f, 0.0f } };

static struct k_spinlock magn_lock;
static float last_magn[3] = { 0.0f, 0.0f, 0.0f };

static int imu_set_last_quat(const struct imu_quat *q)
{
	k_spinlock_key_t key = k_spin_lock(&quat_lock);
	memcpy(&last_quat, q, sizeof(last_quat));
	k_spin_unlock(&quat_lock, key);
	return 0;
}

static int imu_get_last_quat(struct imu_quat *q)
{
	k_spinlock_key_t key = k_spin_lock(&quat_lock);
	memcpy(q, &last_quat, sizeof(*q));
	k_spin_unlock(&quat_lock, key);
	return 0;
}


#if 0
// Telemetry data for debugging
struct telemetry_data {
	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz;
};

static struct telemetry_data telemetry_data;
static struct k_spinlock telemetry_lock;
#endif


// TODO: Do not do any actual fetching here and move it to a separate thread
static void lsm6dsl_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];

	// TODO: Discard the first few samples here
	sensor_sample_fetch(dev);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

	// TODO: Is there a helper to make this conversion nicely?
	// NOTE: In m/s^2
	float ax = sensor_value_to_float(&accel[0]);
	float ay = sensor_value_to_float(&accel[1]);
	float az = sensor_value_to_float(&accel[2]);

	// NOTE: In rad/s
	float gx = sensor_value_to_float(&gyro[0]);
	float gy = sensor_value_to_float(&gyro[1]);
	float gz = sensor_value_to_float(&gyro[2]);

	if (fabsf(ax) >= acc_clip_limit || fabsf(ay) >= acc_clip_limit || fabsf(az) >= acc_clip_limit) {
		LOG_WRN("Accel clipping (limit: %f): %f %f %f", (double)acc_clip_limit, (double)ax, (double)ay, (double)az);
	}

	if (fabsf(gx) >= gyr_clip_limit || fabsf(gy) >= gyr_clip_limit || fabsf(gz) >= gyr_clip_limit) {
		LOG_WRN("Gyro clipping (limit: %f): %f %f %f", (double)gyr_clip_limit, (double)gx, (double)gy, (double)gz);
	}

	k_spinlock_key_t key = k_spin_lock(&magn_lock);
	float mx = last_magn[0];
	float my = last_magn[1];
	float mz = last_magn[2];
	k_spin_unlock(&magn_lock, key);

	sampleFreq = LSM6DSL_SAMPLE_RATE;
	MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
	// NOTE: mx/my is swapped
	// MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, my, mx, mz);


#if 0
	key = k_spin_lock(&telemetry_lock);
	telemetry_data.ax = ax; telemetry_data.ay = ay; telemetry_data.az = az;
	telemetry_data.gx = gx; telemetry_data.gy = gy; telemetry_data.gz = gz;
	telemetry_data.mx = mx; telemetry_data.my = my; telemetry_data.mz = mz;
	k_spin_unlock(&telemetry_lock, key);
#endif


	struct imu_quat new_quat = { .q = { q0, q1, q2, q3 } };

	int ret = imu_set_last_quat(&new_quat);
	if (ret < 0) {
		LOG_ERR("failed to set last quaternion");
	}
}

#if 0
void telemetry_handler(struct k_work *work)
{
	struct telemetry_data d;
	k_spinlock_key_t key = k_spin_lock(&telemetry_lock);
	memcpy(&d, &telemetry_data, sizeof(d));
	k_spin_unlock(&telemetry_lock, key);

	telemetry_uart_printf("a: %8.4f,%8.4f,%8.4f, g: %8.4f,%8.4f,%8.4f, m: %8.4f,%8.4f,%8.4f\r\n",
				d.ax, d.ay, d.az, d.gz, d.gy, d.gz, d.mx, d.my, d.mz);
}
K_WORK_DEFINE(telemetry_work, telemetry_handler);

void telemetry_timer_handler(struct k_timer *timer)
{
	k_work_submit(&telemetry_work);
}
K_TIMER_DEFINE(telemetry_timer, telemetry_timer_handler, NULL);
#endif

#if defined(THEADMOUSE_NUS_DEBUG)
void nus_debug_handler(struct k_work *work)
{
	struct imu_quat cur_quat;
	int ret = imu_get_last_quat(&cur_quat);
	if (ret < 0) {
		LOG_ERR("failed to get last quaternion");
		return;
	}

	float *q = cur_quat.q;
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
K_WORK_DEFINE(nus_debug_work, nus_debug_handler);

void nus_debug_timer_handler(struct k_timer *timer)
{
	k_work_submit(&nus_debug_work);
}
K_TIMER_DEFINE(nus_debug_timer, nus_debug_timer_handler, NULL);
#endif

#if 0
void mag_update_handler(struct k_work *work)
{
	const struct device *const lis2mdl_dev = DEVICE_DT_GET_ONE(st_lis2mdl);

	if (!device_is_ready(lis2mdl_dev)) {
		LOG_ERR("lis2mdl is not ready");
		return;
	}

	int ret = sensor_sample_fetch(lis2mdl_dev);
	if (ret < 0) {
		LOG_ERR("failed to fetch magn sample: %d", ret);
		return;
	}

	struct sensor_value magn[3];
	ret = sensor_channel_get(lis2mdl_dev, SENSOR_CHAN_MAGN_XYZ, magn);
	if (ret < 0) {
		LOG_ERR("failed to get mag value: %d", ret);
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&magn_lock);
	// NOTE: In Gauss
	last_magn[0] = sensor_value_to_float(&magn[0]);
	last_magn[1] = sensor_value_to_float(&magn[1]);
	last_magn[2] = sensor_value_to_float(&magn[2]);
	k_spin_unlock(&magn_lock, key);
}
K_WORK_DEFINE(mag_update_work, mag_update_handler);

void mag_update_timer_handler(struct k_timer *timer)
{
	k_work_submit(&mag_update_work);
}
K_TIMER_DEFINE(mag_update_timer, mag_update_timer_handler, NULL);
#endif

static struct imu_quat hid_last_quat = { .q = { 1.0f, 0.0f, 0.0f, 0.0f } };
void hid_work_handler(struct k_work *work)
{
	float *q_last = hid_last_quat.q;
	float q_conj[4] = { q_last[0], -q_last[1], -q_last[2], -q_last[3] };

	struct imu_quat hid_cur_quat;
	int ret = imu_get_last_quat(&hid_cur_quat);
	if (ret < 0) {
		LOG_ERR("failed to get last quaternion");
		return;
	}

	float *q = hid_cur_quat.q;

	float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
	float pw = q_conj[0], px = q_conj[1], py = q_conj[2], pz = q_conj[3];

	float d_w = qw*pw - qx*px - qy*py - qz*pz;
	float d_x = qw*px + qx*pw + qy*pz - qz*py;
	float d_y = qw*py - qx*pz + qy*pw + qz*px;
	float d_z = qw*pz + qx*py - qy*px + qz*pw;

	float pitch = asinf(2.0f * (d_w * d_y - d_z * d_x));
	float yaw   = atan2f(2.0f * (d_w * d_z + d_x * d_y), 1.0f - 2.0f * (d_y * d_y + d_z * d_z));

	int8_t dx = (int8_t)(-yaw * SENSITIVITY_YAW);
	int8_t dy = (int8_t)(pitch * SENSITIVITY_PITCH);

	//LOG_INF("%d,%d", dx, dy);

	if (dx || dy) {
#if defined(CONFIG_BOARD_THEADMOUSE)
		hog_push_report(0, dx, dy);
#elif defined(CONFIG_BOARD_PUCKJS)
		cur_beacon.dx = dx;
		cur_beacon.dy = dy;
		cur_beacon.seq++;

#if defined(PUCKJS_USE_PERIODIC_ADV)
		bt_le_per_adv_set_data(per_le_adv, le_adv, ARRAY_SIZE(le_adv));
#else
		bt_le_adv_update_data(le_adv, ARRAY_SIZE(le_adv), NULL, 0);
#endif
#endif
	}

	memcpy(&hid_last_quat, &hid_cur_quat, sizeof(hid_last_quat));
}
K_WORK_DEFINE(hid_work, hid_work_handler);

void hid_timer_handler(struct k_timer *timer)
{
	// TODO: Consider using a dedicated workqueue
	k_work_submit(&hid_work);
}
K_TIMER_DEFINE(hid_timer, hid_timer_handler, NULL);

static int lsm6dsl_init()
{
	int ret;
	// TODO: Fetch this via an alias instead
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	if (!device_is_ready(lsm6dsl_dev)) {
		LOG_ERR("lsm6dsl is not ready");
		return -ENODEV;
	}

	struct sensor_value accel_fs;
	sensor_g_to_ms2(LSM6DSL_ACC_FULL_SCALE, &accel_fs);
	ret = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &accel_fs);
	if (ret < 0) {
		LOG_ERR("Failed to set accelerometer range: %d", ret);
		return ret;
	}
	acc_clip_limit = sensor_value_to_float(&accel_fs) * 0.99f;

	struct sensor_value gyro_fs;
	sensor_degrees_to_rad(LSM6DSL_GYR_FULL_SCALE, &gyro_fs);
	ret = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &gyro_fs);
	if (ret < 0) {
		LOG_ERR("Failed to set gyroscope range: %d", ret);
		return ret;
	}
	gyr_clip_limit = sensor_value_to_float(&gyro_fs) * 0.99f;

	struct sensor_value odr_value;
	ret = sensor_value_from_float(&odr_value, LSM6DSL_SAMPLE_RATE);
	if (ret < 0) {
		LOG_ERR("Failed to convert sampling frequency to sensor value: %d", ret);
		return ret;
	}

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

#if 0
static int lis2mdl_init()
{
	int ret;
	// TODO: Fetch this via an alias instead
	const struct device *const lis2mdl_dev = DEVICE_DT_GET_ONE(st_lis2mdl);

	if (!device_is_ready(lis2mdl_dev)) {
		LOG_ERR("lis2mdl is not ready");
		return -ENODEV;
	}

	struct sensor_value odr_value;
	ret = sensor_value_from_float(&odr_value, 100.0f);
	if (ret < 0) {
		LOG_ERR("Failed to convert sampling frequency to sensor value: %d", ret);
		return ret;
	}

	ret = sensor_attr_set(lis2mdl_dev, SENSOR_CHAN_MAGN_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_value);
	if (ret < 0) {
		LOG_ERR("Failed to set magnetometer sampling frequency: %d", ret);
		return ret;
	}

	return 0;
}
#endif

static int led_set_rgb(int r, int g, int b)
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

	ret = ble_init();
	if (ret < 0) {
		LOG_ERR("BLE initialization failed");
	}

#if defined(CONFIG_BOARD_PUCKJS)
	ret = lsm6dsl_init();
	if (ret < 0) {
		LOG_ERR("lsm6dsl initialization failed");
	}
#endif

#if 0
	ret = lis2mdl_init();
	if (ret < 0) {
		LOG_ERR("lsm6dsl initialization failed");
	}
#endif

#if 0
	ret = telemetry_uart_init();
	if (ret < 0) {
		LOG_ERR("telemetry uart initialization failed");
	}
#endif

#if defined(CONFIG_BOARD_THEADMOUSE)
	// k_timer_start(&hid_timer, K_MSEC(0), K_MSEC(10));
#elif defined(CONFIG_BOARD_PUCKJS)
#if defined(PUCKJS_USE_PERIODIC_ADV)
	k_timer_start(&hid_timer, K_MSEC(0), K_MSEC(10));
#else
	k_timer_start(&hid_timer, K_MSEC(0), K_MSEC(100));
#endif
#endif

#if 0
	k_timer_start(&mag_update_timer, K_MSEC(0), K_MSEC(10));
	k_timer_start(&telemetry_timer, K_MSEC(0), K_MSEC(25));
#endif

#if defined(THEADMOUSE_NUS_DEBUG)
	k_timer_start(&nus_debug_timer, K_MSEC(0), K_MSEC(25));
#endif

	while (true) {
		k_msleep(1000);
	}

	return 0;
}

#if defined(CONFIG_SHELL)
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
SHELL_CMD_REGISTER(wipenvs, NULL, "Wipe the NVS settings partition and reboot", wipenvs_handler);
#endif
