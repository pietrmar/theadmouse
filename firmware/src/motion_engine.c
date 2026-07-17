#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <math.h>

#include "common.h"

#include "headmouse_input.h"
#include "motion_engine.h"

#include "MadgwickAHRS/MadgwickAHRS.h"

LOG_MODULE_REGISTER(motion_engine, CONFIG_MOTION_ENGINE_LOG_LEVEL);

#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(mpi_imu))
static const struct device *const dev_imu = DEVICE_DT_GET(DT_CHOSEN(mpi_imu));
#else
static const struct device *const dev_imu = NULL;
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(mpi_mag))
static const struct device *const dev_mag = DEVICE_DT_GET(DT_CHOSEN(mpi_mag));
#else
static const struct device *const dev_mag = NULL;
#endif


// Possible values for the LSM6DSL from the datasheet:
//   1.6, 12.5, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664
#define IMU_SAMPLING_FREQUENCY	208.0f

// Posible values for the LIS2MDL from the datasheet:
//   10, 20, 50, 100
#define MAG_SAMPLING_FREQUENCY	50.0f

// MAG_DECIM_FACTOR = ceil(IMU_SAMPLING_FREQUENCY / MAG_SAMPLING_FREQUENCY)
#define MAG_DECIM_FACTOR	((uint32_t)((IMU_SAMPLING_FREQUENCY + MAG_SAMPLING_FREQUENCY - 1) / MAG_SAMPLING_FREQUENCY))


static atomic_t imu_trigger_count = ATOMIC_INIT(0);

static atomic_t accel_clip_count = ATOMIC_INIT(0);
static atomic_t gyro_clip_count = ATOMIC_INIT(0);
float accel_clip_limit = 0.0f;
float gyro_clip_limit = 0.0f;

static K_SEM_DEFINE(imu_trigger_sem, 0, 1);
static void imu_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	atomic_inc(&imu_trigger_count);
	k_sem_give(&imu_trigger_sem);
}


// To prevent aliasing effects, the IMU thread calculates the delta motion between samples and
// accumulates them in the `imu_acuum_[x|y]` accumulator. The HID thread will fetch and clear
// this accumulator at a lower rate when needed. This ensures we do not "lose" position updates
// and every IMU update was correctly accumulated.
static float imu_accum_x = 0.0f;
static float imu_accum_y = 0.0f;
static struct k_spinlock imu_accum_lock;

// Posible values from the data-sheet in g:
//  2, 4, 8, 16
#define LSM6DSL_ACC_FULL_SCALE	2

// Possible values from the data-sheet deg/s:
//  125, 250, 500, 1000, 2000
#define LSM6DSL_GYR_FULL_SCALE	500


extern int slot_manager_set_rgb_led_color(int r, int g, int b);

static void imu_thread(void *, void *, void *)
{
	int ret;

	float ax_prev = 0.0f;
	float ay_prev = 0.0f;
	float az_prev = 0.0f;

	float gx_prev = 0.0f;
	float gy_prev = 0.0f;
	float gz_prev = 0.0f;

	float q_p[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

	sampleFreq = IMU_SAMPLING_FREQUENCY;

	const uint32_t log_sample_reset_val = IMU_SAMPLING_FREQUENCY / 4;
	uint32_t log_sample_counter = log_sample_reset_val;

	uint32_t mag_read_counter = MAG_DECIM_FACTOR;
	const uint32_t mag_log_reset_val = 40;
	uint32_t mag_log_counter = mag_log_reset_val;

	float gx_p = 0.0f;
	float gy_p = 0.0f;
	float gz_p = 0.0f;

	bool is_still = true;
	while (true) {
		k_sem_take(&imu_trigger_sem, K_FOREVER);

		int64_t t_start = k_uptime_ticks();

		// TODO: Verify that we are actually fetching correctly both fresh samples
		// from the IMU and not that one of them is stale.
		ret = sensor_sample_fetch_chan(dev_imu, SENSOR_CHAN_ALL);
		if (ret < 0) {
			LOG_ERR("Failed to fetch IMU sample: %d", ret);
			continue;
		}

		// Fetch accelerometer data
		struct sensor_value acc_val[3];
		sensor_channel_get(dev_imu, SENSOR_CHAN_ACCEL_XYZ, acc_val);	// In m/s²

		// Fetch gyro data
		struct sensor_value gyro_val[3];
		sensor_channel_get(dev_imu, SENSOR_CHAN_GYRO_XYZ, gyro_val);	// In rad/s

		float ax = sensor_value_to_float(&acc_val[0]);
		float ay = sensor_value_to_float(&acc_val[1]);
		float az = sensor_value_to_float(&acc_val[2]);

		float gx = sensor_value_to_float(&gyro_val[0]);
		float gy = sensor_value_to_float(&gyro_val[1]);
		float gz = sensor_value_to_float(&gyro_val[2]);

		float dgx = fabsf(gx - gx_p);
		float dgy = fabsf(gy - gy_p);
		float dgz = fabsf(gz - gz_p);

		gx_p = gx;
		gy_p = gy;
		gz_p = gz;

#if 0
		mag_read_counter -= 1;
		if (mag_read_counter == 0) {
			mag_read_counter = MAG_DECIM_FACTOR;

			ret = sensor_sample_fetch_chan(dev_mag, SENSOR_CHAN_MAGN_XYZ);
			if (ret < 0) {
				LOG_ERR("Failed to fetch magnetometer sample: %d", ret);
			}

			struct sensor_value mag_val[3];
			sensor_channel_get(dev_mag, SENSOR_CHAN_MAGN_XYZ, mag_val);	// In Gauss

			float mx = sensor_value_to_float(&mag_val[0]);
			float my = sensor_value_to_float(&mag_val[1]);
			float mz = sensor_value_to_float(&mag_val[2]);

			static float mag_min_x = 9999.0f; static float mag_max_x = -9999.0f;
			static float mag_min_y = 9999.0f; static float mag_max_y = -9999.0f;
			static float mag_min_z = 9999.0f; static float mag_max_z = -9999.0f;

			if (mx < mag_min_x) mag_min_x = mx;
			if (mx > mag_max_x) mag_max_x = mx;

			if (my < mag_min_y) mag_min_y = my;
			if (my > mag_max_y) mag_max_y = my;

			if (mz < mag_min_z) mag_min_z = mz;
			if (mz > mag_max_z) mag_max_z = mz;

			float offset_x = (mag_max_x - mag_min_x) / 2.0f;
			float offset_y = (mag_max_y - mag_min_y) / 2.0f;
			float offset_z = (mag_max_z - mag_min_z) / 2.0f;

			// Convert to uT
			float cal_mx = (mx - offset_x) * 100.0f;
			float cal_my = (my - offset_y) * 100.0f;
			float cal_mz = (mz - offset_z) * 100.0f;

			float mag_norm = sqrtf(cal_mx*cal_mx + cal_my*cal_my + cal_mz*cal_mz);

			mag_log_counter -= 1;
			if (mag_log_counter == 0) {
				mag_log_counter = mag_log_reset_val;
				LOG_INF("mag: [%f, %f, %f]", mx, my, mz);
				LOG_INF("min: [%f, %f, %f]", mag_min_x, mag_min_y, mag_min_z);
				LOG_INF("max: [%f, %f, %f]", mag_max_x, mag_max_y, mag_max_z);
				LOG_INF("off: [%f, %f, %f]", offset_x, offset_y, offset_z);
				LOG_INF("cal: [%f, %f, %f]", cal_mx, cal_my, cal_mz);
				LOG_INF("|mag|: %f", mag_norm);
			}
		}
#endif

		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);

		// TODO: Change this to calculate dq = conj(q_prev) * q_curr
		float dq3 = q3 * q_p[0] - q0 * q_p[3] - q1 * q_p[2] + q2 * q_p[1];
		float dq2 = q2 * q_p[0] - q0 * q_p[2] + q1 * q_p[3] - q3 * q_p[1];

		// float raw_dx = 2.0f * dq3;
		// float raw_dy = 2.0f * dq2;
		float raw_dx = 0.0f;
		float raw_dy = 0.0f;

		q_p[0] = q0;
		q_p[1] = q1;
		q_p[2] = q2;
		q_p[3] = q3;

		ax_prev = ax; ay_prev = ay; az_prev = az;
		gx_prev = gx; gy_prev = gy; gz_prev = gz;


		k_spinlock_key_t key = k_spin_lock(&imu_accum_lock);
		imu_accum_x -= raw_dx;
		imu_accum_y += raw_dy;
		k_spin_unlock(&imu_accum_lock, key);

		int64_t t_end = k_uptime_ticks();

		log_sample_counter -= 1;
		if (log_sample_counter == 0) {
			log_sample_counter = log_sample_reset_val;
			LOG_DBG("gyro: [%f, %f, %f]", gx, gy, gz);
			LOG_DBG("gyro': [%f, %f, %f]", dgx, dgy, dgz);
			//telemetry_uart_printf("%f, %f, %f\r\n", (double)dgx, (double)dgy, (double)dgz);


			// LOG_DBG("accel_delta: %f", accel_delta);
			// LOG_DBG("loop time: %lld us", k_ticks_to_us_near64(t_end - t_start));
		}
	}
}
// NOTE: `imu_trigger_handler()` should run at `-2`, and the `imu_thread` then one
// prio lower at `-1`.
K_THREAD_DEFINE(imu_tid, 2048, imu_thread, NULL, NULL, NULL, K_PRIO_COOP(1), 0, 0);

struct motion_config {
	enum mouse_mode mouse_mode;

	float acceleration[2];
	float deadzone[2];
	float maximum_speed;
	float acceleration_time;
};

struct motion_state {
	float absolute_pos[2];
};

struct motion_ctx {
	struct motion_config cfg;
	struct motion_state state;
	struct k_spinlock lock;
};

static struct motion_ctx ctx = {
	.cfg = {
		.mouse_mode = MOUSE_MODE_MOUSE,

		.acceleration = { 128.0f, 128.0f },
		.deadzone = { 5.0f, 5.0f },
		.maximum_speed = 50.0f,
		.acceleration_time = 50.0f,
	},
};

enum mouse_mode motion_engine_get_mouse_mode(void)
{
	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	enum mouse_mode m = ctx.cfg.mouse_mode;
	k_spin_unlock(&ctx.lock, key);

	return m;
}

int motion_engine_set_mouse_mode(enum mouse_mode m)
{
	if (m >= MOUSE_MODE_MAX)
		return -EINVAL;

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.cfg.mouse_mode = m;
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

int motion_engine_get_absolute_pos(float *x, float *y)
{
	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	*x = ctx.state.absolute_pos[AXIS_X];
	*y = ctx.state.absolute_pos[AXIS_Y];
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

// TODO: Depending on what kind of orientation algorithm we use here it might make
// sense to reset/reinitialize the IMU/orientation estimation state too.
void motion_engine_reset_absolute_pos(void)
{
	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.absolute_pos[AXIS_X] = 0.0f;
	ctx.state.absolute_pos[AXIS_Y] = 0.0f;
	k_spin_unlock(&ctx.lock, key);
}


int motion_engine_set_acceleration(float v, enum axis axis)
{
	if (axis >= ARRAY_SIZE(ctx.cfg.acceleration))
		return -EINVAL;

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.cfg.acceleration[axis] = v;
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

float motion_engine_get_acceleration(enum axis axis)
{
	if (axis >= ARRAY_SIZE(ctx.cfg.acceleration))
		return -EINVAL;

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	float v = ctx.cfg.acceleration[axis];
	k_spin_unlock(&ctx.lock, key);

	return v;
}

int motion_engine_set_deadzone(float v, enum axis axis)
{
	if (axis >= ARRAY_SIZE(ctx.cfg.deadzone))
		return -EINVAL;

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.cfg.deadzone[axis] = v;
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

float motion_engine_get_deadzone(enum axis axis)
{
	if (axis >= ARRAY_SIZE(ctx.cfg.deadzone))
		return -EINVAL;

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	float v = ctx.cfg.deadzone[axis];
	k_spin_unlock(&ctx.lock, key);

	return v;
}

static const struct device *const btn_dev = DEVICE_DT_GET(DT_CHOSEN(mpi_buttons));

K_SEM_DEFINE(hid_report_sem, 0, 1);
static void hid_report_cb(struct k_timer *timer)
{
	k_sem_give(&hid_report_sem);
}
K_TIMER_DEFINE(hid_report_timer, hid_report_cb, NULL);

#define HID_REPORT_INTERVAL K_MSEC(10)

static void hid_report_thread(void *, void *, void *)
{
	// TODO: Reset all of these on mode change
	float remainder_x = 0.0f;
	float remainder_y = 0.0f;
	bool alt_mode_hits[4] = { false };

	k_timer_start(&hid_report_timer, HID_REPORT_INTERVAL, HID_REPORT_INTERVAL);

	while (true) {
		k_sem_take(&hid_report_sem, K_FOREVER);

		k_spinlock_key_t key = k_spin_lock(&imu_accum_lock);
		float raw_dx = imu_accum_x;
		float raw_dy = imu_accum_y;
		imu_accum_x = 0.0f;
		imu_accum_y = 0.0f;
		k_spin_unlock(&imu_accum_lock, key);

		// Make a "snapshot" of the config
		key = k_spin_lock(&ctx.lock);
		struct motion_config cfg = ctx.cfg;
		k_spin_unlock(&ctx.lock, key);

		// TODO: Do something more advanced than just a multiplication by the acceleration
		float move_dx = raw_dx * cfg.acceleration[AXIS_X];
		float move_dy = raw_dy * cfg.acceleration[AXIS_Y];

		// Update the absolute position and also make a snapshot of it for further processing
		key = k_spin_lock(&ctx.lock);
		ctx.state.absolute_pos[AXIS_X] += move_dx;
		ctx.state.absolute_pos[AXIS_Y] += move_dy;

		struct motion_state state = ctx.state;
		k_spin_unlock(&ctx.lock, key);

		if (cfg.mouse_mode == MOUSE_MODE_MOUSE) {
			// Because we can only transmit whole pixel values and thus we are rounding
			// to the nearest integer. So there might be a float reminder left which is
			// up to half a pixel. This mechanism makes sure that this remainder is not
			// lost and correctly included in the next HID update calculation.
			// TODO: Figure out how to keep/handle this correctly across mode changes
			remainder_x += move_dx;
			remainder_y += move_dy;

			int16_t idx = roundf(remainder_x);
			int16_t idy = roundf(remainder_y);

			if (idx != 0 || idy != 0)
				hm_input_report_mouse_move(idx, idy, K_FOREVER);

			remainder_x -= idx;
			remainder_y -= idy;
		} else if (cfg.mouse_mode == MOUSE_MODE_ALT) {
			bool hit[4] = { false };

			// TODO: Probably we would want to add a bit of hysteresis here

			if (state.absolute_pos[AXIS_X] > cfg.deadzone[AXIS_X])
				hit[DIR_RIGHT] = true;
			else if (state.absolute_pos[AXIS_X] < -cfg.deadzone[AXIS_X])
				hit[DIR_LEFT] = true;

			if (state.absolute_pos[AXIS_Y] > cfg.deadzone[AXIS_Y])
				hit[DIR_DOWN] = true;
			else if (state.absolute_pos[AXIS_Y] < -cfg.deadzone[AXIS_Y])
				hit[DIR_UP] = true;

			static const uint16_t zephyr_codes[4] = { INPUT_KEY_UP, INPUT_KEY_DOWN, INPUT_KEY_LEFT, INPUT_KEY_RIGHT };

			for (int i = 0; i < 4; i++) {
				bool is_hit = hit[i];

				if (alt_mode_hits[i] != is_hit) {
					alt_mode_hits[i] = is_hit;
					input_report_key(btn_dev, zephyr_codes[i], is_hit ? 1 : 0, true, K_FOREVER);
				}
			}
		}
	}
}
K_THREAD_DEFINE(hid_report_tid, 2048, hid_report_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, 0);

static void imu_monitor_cb(struct k_timer *timer)
{
	static uint32_t last_trigger_count = 0;
	uint32_t trigger_count = atomic_get(&imu_trigger_count);
	uint32_t trigger_delta = trigger_count - last_trigger_count;
	last_trigger_count = trigger_count;

	uint32_t accel_clip_cnt = atomic_get(&accel_clip_count);
	uint32_t gyro_clip_cnt = atomic_get(&gyro_clip_count);

	// TODO: Does this handle the overflow correctly?
	if (trigger_delta == 0) {
		LOG_ERR("IMU trigger stall, trigger: %u (total: %u), clip: a: %u, g: %u", trigger_delta, trigger_count, accel_clip_cnt, gyro_clip_cnt);
	} else {
		// LOG_DBG("trigger: %u (total: %u), clip: a: %u, g: %u", trigger_delta, trigger_count, accel_clip_cnt, gyro_clip_cnt);
	}
}
K_TIMER_DEFINE(imu_monitor_timer, imu_monitor_cb, NULL);

int motion_engine_init(void)
{
	if (dev_imu == NULL || !device_is_ready(dev_imu)) {
		LOG_ERR("IMU is not ready");
		return -ENODEV;
	}

	if (dev_mag == NULL || !device_is_ready(dev_mag)) {
		LOG_ERR("Magnetometer is not ready");
		return -ENODEV;
	}

	int ret;

	struct sensor_value accel_fs;
	sensor_g_to_ms2(LSM6DSL_ACC_FULL_SCALE, &accel_fs);
	accel_clip_limit = sensor_value_to_float(&accel_fs) * 0.98f;
	ret = sensor_attr_set(dev_imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &accel_fs);
	if (ret < 0) {
		LOG_ERR("Failed to set accelerometer range: %d", ret);
		return ret;
	}

	struct sensor_value gyro_fs;
	sensor_degrees_to_rad(LSM6DSL_GYR_FULL_SCALE, &gyro_fs);
	gyro_clip_limit = sensor_value_to_float(&gyro_fs) * 0.98f;
	ret = sensor_attr_set(dev_imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &gyro_fs);
	if (ret < 0) {
		LOG_ERR("Failed to set gyroscope range: %d", ret);
		return ret;
	}

	struct sensor_value odr_value;
	ret = sensor_value_from_float(&odr_value, MAG_SAMPLING_FREQUENCY);
	if (ret < 0) {
		LOG_ERR("Failed to convert sampling frequency to sensor value: %d", ret);
		return ret;
	}

	ret = sensor_attr_set(dev_mag, SENSOR_CHAN_MAGN_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_value);
	if (ret < 0) {
		LOG_ERR("Failed to set magnetometer sampling frequency: %d", ret);
		return ret;
	}


	ret = sensor_value_from_float(&odr_value, IMU_SAMPLING_FREQUENCY);
	if (ret < 0) {
		LOG_ERR("Failed to convert sampling frequency to sensor value: %d", ret);
		return ret;
	}

	ret = sensor_attr_set(dev_imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_value);
	if (ret < 0) {
		LOG_ERR("Failed to set accelerometer sampling frequency: %d", ret);
		return ret;
	}

	ret = sensor_attr_set(dev_imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_value);
	if (ret < 0) {
		LOG_ERR("Failed to set gyroscope sampling frequency: %d", ret);
		return ret;
	}

	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	ret = sensor_trigger_set(dev_imu, &trig, imu_trigger_handler);
	if (ret < 0) {
		LOG_ERR("Failed to set IMU trigger: %d", ret);
		return ret;
	}

	k_timer_start(&imu_monitor_timer, K_SECONDS(1), K_SECONDS(1));

	LOG_INF("Motion engine init done");
	return 0;
}

int motion_engine_suspend(void)
{
	LOG_INF("Suspending");

	k_timer_stop(&hid_report_timer);
	k_timer_stop(&imu_monitor_timer);

	int ret = pm_device_action_run(dev_imu, PM_DEVICE_ACTION_SUSPEND);
	if (ret < 0) {
		LOG_ERR("Failed to suspend the IMU: %d", ret);
	}

	ret = pm_device_action_run(dev_mag, PM_DEVICE_ACTION_SUSPEND);
	if (ret < 0) {
		LOG_ERR("Failed to suspend the magnetometer: %d", ret);
	}

	return 0;
}

int motion_engine_resume(void)
{
	LOG_INF("Resuming");

	int ret = pm_device_action_run(dev_mag, PM_DEVICE_ACTION_RESUME);
	if (ret < 0) {
		LOG_ERR("Failed to resume the magnetometer: %d", ret);
	}

	ret = pm_device_action_run(dev_imu, PM_DEVICE_ACTION_RESUME);
	if (ret < 0) {
		LOG_ERR("Failed to resume the IMU: %d", ret);
	}

	k_timer_start(&imu_monitor_timer, K_SECONDS(1), K_SECONDS(1));
	k_timer_start(&hid_report_timer, HID_REPORT_INTERVAL, HID_REPORT_INTERVAL);

	return 0;
}
