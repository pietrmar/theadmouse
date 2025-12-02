#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <zsl/orientation/orientation.h>

#include "headmouse_input.h"
#include "telemetry_uart.h"

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
#define IMU_SAMPLING_FREQUENCY	416.0f

// Posible values for the LIS2MDL from the datasheet:
//   10, 20, 50, 100
#define MAG_SAMPLING_FREQUENCY	50.0f

// MAG_DECIM_FACTOR = ceil(IMU_SAMPLING_FREQUENCY / MAG_SAMPLING_FREQUENCY)
#define MAG_DECIM_FACTOR	((uint32_t)((IMU_SAMPLING_FREQUENCY + MAG_SAMPLING_FREQUENCY - 1) / MAG_SAMPLING_FREQUENCY))


static atomic_t imu_trigger_count = ATOMIC_INIT(0);
static atomic_t imu_fetch_count = ATOMIC_INIT(0);

static K_SEM_DEFINE(imu_trigger_sem, 0, 1);
static void imu_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	atomic_inc(&imu_trigger_count);
	k_sem_give(&imu_trigger_sem);
}


static struct zsl_quat cur_orient_q = {
	.r = 1.0f, .i = 0.0f, .j = 0.0f, .k = 0.0f
};
static struct k_spinlock cur_orient_lock;

#define TELEMETRY_DECIM_FACTOR	10
static inline int feed_telemetry(uint32_t sample_id, const struct zsl_vec *a, const struct zsl_vec *g, const struct zsl_vec *m, const struct zsl_quat *q, float t)
{
	return telemetry_uart_printf("S,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
			sample_id,
			(double)a->data[0], (double)a->data[1], (double)a->data[2],
			(double)g->data[0], (double)g->data[1], (double)g->data[2],
			(double)m->data[0], (double)m->data[1], (double)m->data[2],
			(double)q->r, (double)q->i, (double)q->j, (double)q->k,
			(double)t);
}

// NOTE: For some reason the zscilib orientation fuse API does not work properly,
// at least the Madgwick algorithm, so do not use it for now and use the good old
// MadgwickAHRS library instead which seems to work.
// #define USE_ZSL_FUSE

static void imu_thread(void *, void *, void *)
{
	int ret;

#if defined(USE_ZSL_FUSE)
	static struct zsl_fus_madg_cfg madg_cfg = {
		.beta = 0.1,
	};

	static struct zsl_fus_drv madgwick_drv = {
		.init_handler = zsl_fus_madg_init,
		.feed_handler = zsl_fus_madg_feed,
		.error_handler = zsl_fus_madg_error,
		.config = &madg_cfg,
	};

	struct zsl_fus_drv *fus_drv = &madgwick_drv;
	ret = fus_drv->init_handler(IMU_SAMPLING_FREQUENCY, fus_drv->config);
	if (ret < 0) {
		LOG_ERR("init_handler() failed: %d", ret);
		return;
	}
#else
	beta = 0.1;
	sampleFreq = IMU_SAMPLING_FREQUENCY;
#endif


	// For the telemetry stream we need to keep a copy of the last mag value read
	// from our loop as the MAG_DECIM_FACTOR and TELEMETRY_DECIM_FACTOR are not the
	// same and "unlocked";
	bool telemetry_mag_valid = false;
	ZSL_VECTOR_DEF(telemetry_mag, 3);
	zsl_vec_init(&telemetry_mag);

	ZSL_VECTOR_DEF(gyro_bias, 3);
	zsl_vec_init(&gyro_bias);

	uint32_t sample_idx = 0;
	while (true) {

		k_sem_take(&imu_trigger_sem, K_FOREVER);
		ret = sensor_sample_fetch_chan(dev_imu, SENSOR_CHAN_ALL);
		if (ret < 0) {
			LOG_ERR("Failed to fetch IMU sample: %d", ret);
			continue;
		}
		atomic_inc(&imu_fetch_count);
		sample_idx++;

		// Fetch accelerometer data
		struct sensor_value acc_val[3];
		sensor_channel_get(dev_imu, SENSOR_CHAN_ACCEL_XYZ, acc_val);	// In m/s²
		ZSL_VECTOR_DEF(raw_acc, 3);
		raw_acc_vec[0] = sensor_value_to_float(&acc_val[0]);
		raw_acc_vec[1] = sensor_value_to_float(&acc_val[1]);
		raw_acc_vec[2] = sensor_value_to_float(&acc_val[2]);

		// Fetch gyro data
		struct sensor_value gyro_val[3];
		sensor_channel_get(dev_imu, SENSOR_CHAN_GYRO_XYZ, gyro_val);	// In rad/s
		ZSL_VECTOR_DEF(raw_gyr, 3);
		raw_gyr_vec[0] = sensor_value_to_float(&gyro_val[0]);
		raw_gyr_vec[1] = sensor_value_to_float(&gyro_val[1]);
		raw_gyr_vec[2] = sensor_value_to_float(&gyro_val[2]);


		bool mag_fetched = false;
		ZSL_VECTOR_DEF(raw_mag, 3);
		if ((sample_idx % MAG_DECIM_FACTOR) == 0) {
			ret = sensor_sample_fetch_chan(dev_mag, SENSOR_CHAN_MAGN_XYZ);
			if (ret < 0) {
				LOG_ERR("Failed to fetch magnetometer sample: %d", ret);
			}
			struct sensor_value mag_val[3];
			sensor_channel_get(dev_mag, SENSOR_CHAN_MAGN_XYZ, mag_val);	// In Gauss
			raw_mag_vec[0] = sensor_value_to_float(&mag_val[0]);
			raw_mag_vec[1] = sensor_value_to_float(&mag_val[1]);
			raw_mag_vec[2] = sensor_value_to_float(&mag_val[2]);
			mag_fetched = true;

			telemetry_mag_vec[0] = raw_mag_vec[0];
			telemetry_mag_vec[1] = raw_mag_vec[1];
			telemetry_mag_vec[2] = raw_mag_vec[2];
			telemetry_mag_valid = true;
		}

		if ((sample_idx % TELEMETRY_DECIM_FACTOR) == 0) {
			struct sensor_value temp;
			ret = sensor_channel_get(dev_imu, SENSOR_CHAN_DIE_TEMP, &temp);
			if (ret < 0)
				LOG_ERR("Failed to get IMU temperature: %d", ret);

			float t = sensor_value_to_float(&temp);
			if (telemetry_mag_valid) {
				telemetry_mag_valid = false;
			} else {
				telemetry_mag_vec[0] = 0.0f;
				telemetry_mag_vec[1] = 0.0f;
				telemetry_mag_vec[2] = 0.0f;
			}

			k_spinlock_key_t key = k_spin_lock(&cur_orient_lock);
			struct zsl_quat q = cur_orient_q;
			k_spin_unlock(&cur_orient_lock, key);

			feed_telemetry(sample_idx, &raw_acc, &raw_gyr, &telemetry_mag, &q, t);
		}

		zsl_real_t acc_norm = zsl_vec_norm(&raw_acc);
		zsl_real_t gyr_norm = zsl_vec_norm(&raw_gyr);
		// TODO: Make use of the magnetometer
		zsl_real_t mag_norm = zsl_vec_norm(&raw_mag);

		bool acc_static = false;
		if (acc_norm > 9.0f && acc_norm < 11.0f)
			acc_static = true;

		bool gyr_static = false;
		if (gyr_norm < 0.1f)
			gyr_static = true;

		ZSL_VECTOR_DEF(accel, 3);
		zsl_vec_copy(&accel, &raw_acc);
		zsl_vec_scalar_mult(&accel, 1.0f / acc_norm);

		if (acc_static && gyr_static) {
			const zsl_real_t alpha = (1.0f / IMU_SAMPLING_FREQUENCY) / 5.0f;

			// gyro_bias = gyro_bias * (1.0 - alpha) + raw_gyr * alpha

			// tmp1 = gyro_bias * (1 - alpha)
			ZSL_VECTOR_DEF(tmp1, 3);
			zsl_vec_copy(&tmp1, &gyro_bias);
			zsl_vec_scalar_mult(&tmp1, 1.0f - alpha);

			// tmp2 = raw_gyr * alpha
			ZSL_VECTOR_DEF(tmp2, 3);
			zsl_vec_copy(&tmp2, &raw_gyr);
			zsl_vec_scalar_mult(&tmp2, alpha);

			// gyro_bias = tmp1 + tmp2
			zsl_vec_add(&tmp1, &tmp2, &gyro_bias);
		}
		ZSL_VECTOR_DEF(gyro, 3);
		zsl_vec_sub(&raw_gyr, &gyro_bias, &gyro);

#if defined(USE_ZSL_FUSE)
		k_spinlock_key_t key = k_spin_lock(&cur_orient_lock);
		struct zsl_quat q = cur_orient_q;
		k_spin_unlock(&cur_orient_lock, key);

		fus_drv->feed_handler(&accel, NULL, &gyro, NULL, &q, fus_drv->config);

		key = k_spin_lock(&cur_orient_lock);
		cur_orient_q = q;
		k_spin_unlock(&cur_orient_lock, key);
#else
		MadgwickAHRSupdateIMU(gyro_vec[0], gyro_vec[1], gyro_vec[2], accel_vec[0], accel_vec[1], accel_vec[2]);

		k_spinlock_key_t key = k_spin_lock(&cur_orient_lock);
		cur_orient_q.r = q0;
		cur_orient_q.i = q1;
		cur_orient_q.j = q2;
		cur_orient_q.k = q3;
		k_spin_unlock(&cur_orient_lock, key);
#endif

		if ((sample_idx % (uint32_t)IMU_SAMPLING_FREQUENCY) == 0) {
			LOG_DBG("gyro bias: [%f, %f, %f]", (double)gyro_bias_vec[0], (double)gyro_bias_vec[1], (double)gyro_bias_vec[2]);
		}
	}
}
// NOTE: `imu_trigger_handler()` should run at `-2`, and the `imu_thread` then one
// prio lower at `-1`.
K_THREAD_DEFINE(imu_tid, 2048, imu_thread, NULL, NULL, NULL, K_PRIO_COOP(1), 0, 0);

static void hid_mouse_thread(void *, void *, void *)
{
	static struct zsl_quat q_prev = {
		.r = 1.0f, .i = 0.0f, .j = 0.0f, .k = 0.0f
	};

	while (true) {
		k_sleep(K_MSEC(10));

		k_spinlock_key_t key = k_spin_lock(&cur_orient_lock);
		struct zsl_quat q_curr = cur_orient_q;
		k_spin_unlock(&cur_orient_lock, key);

		// Compute delta rotation:
		//  q_delta = q_prev^(-1) * q_curr
		struct zsl_quat q_prev_conj;
		zsl_quat_conj(&q_prev, &q_prev_conj);

		struct zsl_quat q_delta;
		zsl_quat_mult(&q_prev_conj, &q_curr, &q_delta);

		q_prev = q_curr;

		struct zsl_euler e;
		zsl_quat_to_euler(&q_delta, &e);

		float raw_dx = -800.0f * e.z;
		float raw_dy =  800.0f * e.y;

		hm_input_report_mouse_move(raw_dx, raw_dy, K_FOREVER);
	}
}
K_THREAD_DEFINE(hid_mouse_tid, 2048, hid_mouse_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, 0);

static void imu_monitor_cb(struct k_timer *timer)
{
	static uint32_t last_trigger_count = 0;
	uint32_t trigger_count = atomic_get(&imu_trigger_count);
	uint32_t trigger_delta = trigger_count - last_trigger_count;
	last_trigger_count = trigger_count;

	static uint32_t last_fetch_count = 0;
	uint32_t fetch_count = atomic_get(&imu_fetch_count);
	uint32_t fetch_delta = fetch_count - last_fetch_count;
	last_fetch_count = fetch_count;

	// TODO: Does this handle the overflow correctly?
	if (trigger_delta == 0) {
		LOG_ERR("IMU trigger stall, trigger: %u (total: %u), fetch: %u (total: %u)", trigger_delta, trigger_count, fetch_delta, fetch_count);
	} else {
		LOG_DBG("trigger: %u (total: %u), fetch: %u (total: %u)", trigger_delta, trigger_count, fetch_delta, fetch_count);
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

// Posible values from the data-sheet in g:
//  2, 4, 8, 16
#define LSM6DSL_ACC_FULL_SCALE	4

// Possible values from the data-sheet deg/s:
//  125, 250, 500, 1000, 2000
#define LSM6DSL_GYR_FULL_SCALE	500

	struct sensor_value accel_fs;
	sensor_g_to_ms2(LSM6DSL_ACC_FULL_SCALE, &accel_fs);
	ret = sensor_attr_set(dev_imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &accel_fs);
	if (ret < 0) {
		LOG_ERR("Failed to set accelerometer range: %d", ret);
		return ret;
	}

	struct sensor_value gyro_fs;
	sensor_degrees_to_rad(LSM6DSL_GYR_FULL_SCALE, &gyro_fs);
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
