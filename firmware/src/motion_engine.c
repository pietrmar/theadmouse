#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

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


static struct zsl_quat cur_orient_q = {
	.r = 1.0f, .i = 0.0f, .j = 0.0f, .k = 0.0f
};
static struct k_spinlock cur_orient_lock;

// The plaintext telemetry takes up to 1.9 ms to format and send out
// TODO: Make this a Kconfig option
#define TELEMETRY_PLAINTEXT	0
#define TELEMETRY_DECIM_FACTOR	10

static inline int feed_telemetry(uint32_t sample_id, const struct zsl_vec *a, const struct zsl_vec *g, const struct zsl_vec *m, const struct zsl_quat *q, float t)
{
#if !TELEMETRY_PLAINTEXT
#define TELEMETRY_PAYLOAD_LEN	60	// 1*4 (uint32_t) + 14*4 (float)

	//  1 * 4 bytes (uint32_t), sample_id
	// 14 * 4 bytes (float), a(3), g(3), m(3), q(4), t(1)
	const size_t telemetry_payload_len = 60;
	// 2 (sync) + 1 (version) + 2 (length) + payload + 1 (checksum)
	const size_t buffer_size = 2 + 1 + 2 + telemetry_payload_len + 1;
	uint8_t buf[buffer_size];
	size_t idx = 0;

	// sync bytes
	buf[idx++] = 0xaa;
	buf[idx++] = 0x55;

	// Version
	buf[idx++] = 0x01;

	// Payload length
	sys_put_le16(TELEMETRY_PAYLOAD_LEN, &buf[idx]);
	idx += 2;

	// Sample ID
	sys_put_le32(sample_id, &buf[idx]);
	idx +=4;

#define PACK_F32(_buf, _idx, _value) do {		\
	float _v = (_value);				\
	memcpy(&(_buf)[(_idx)], &_v, sizeof(_v));	\
	(_idx) += sizeof(_v);				\
} while (0)

	PACK_F32(buf, idx, a->data[0]);
	PACK_F32(buf, idx, a->data[1]);
	PACK_F32(buf, idx, a->data[2]);

	PACK_F32(buf, idx, g->data[0]);
	PACK_F32(buf, idx, g->data[1]);
	PACK_F32(buf, idx, g->data[2]);

	PACK_F32(buf, idx, m->data[0]);
	PACK_F32(buf, idx, m->data[1]);
	PACK_F32(buf, idx, m->data[2]);

	PACK_F32(buf, idx, q->r);
	PACK_F32(buf, idx, q->i);
	PACK_F32(buf, idx, q->j);
	PACK_F32(buf, idx, q->k);

	PACK_F32(buf, idx, t);

#undef PACK_F32

	// Sanity check before the checksum
	if (idx != (buffer_size - 1)) {
		// TODO: Maybe do an assert here
		return -EFAULT;
	}

	uint8_t checksum = 0;
	for (size_t i = 2; i < idx; i++) {
		checksum = (uint8_t)(checksum + buf[i]);
	}
	buf[idx++] = checksum;

	return telemetry_uart_write(buf, idx);
#else
	// NOTE: This is slow, like up to 1.9 ms slow, most likely becasue of float formatting
	return telemetry_uart_printf("S,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
			sample_id,
			(double)a->data[0], (double)a->data[1], (double)a->data[2],
			(double)g->data[0], (double)g->data[1], (double)g->data[2],
			(double)m->data[0], (double)m->data[1], (double)m->data[2],
			(double)q->r, (double)q->i, (double)q->j, (double)q->k,
			(double)t);
#endif // !TELEMETRY_PLAINTEXT
}


// Posible values from the data-sheet in g:
//  2, 4, 8, 16
#define LSM6DSL_ACC_FULL_SCALE	8

// Possible values from the data-sheet deg/s:
//  125, 250, 500, 1000, 2000
#define LSM6DSL_GYR_FULL_SCALE	1000


static void imu_thread(void *, void *, void *)
{
	int ret;

	// Madgwick setup
	beta = 0.1;
	sampleFreq = IMU_SAMPLING_FREQUENCY;


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

		int64_t t_start = k_uptime_ticks();

		// TODO: Verify that we are actually fetching correctly both fresh samples
		// from the IMU and not that one of them is stale.
		ret = sensor_sample_fetch_chan(dev_imu, SENSOR_CHAN_ALL);
		if (ret < 0) {
			LOG_ERR("Failed to fetch IMU sample: %d", ret);
			continue;
		}
		sample_idx++;

		bool mag_fetched = false;
		if ((sample_idx % MAG_DECIM_FACTOR) == 0) {
			ret = sensor_sample_fetch_chan(dev_mag, SENSOR_CHAN_MAGN_XYZ);
			if (ret < 0) {
				LOG_ERR("Failed to fetch magnetometer sample: %d", ret);
			}
			mag_fetched = true;
		}

		// TODO: Drop 50 Mag samples or determine stability
		// TODO: Drop 200 IMU samples or determine stability

		// Fetch accelerometer data
		struct sensor_value acc_val[3];
		sensor_channel_get(dev_imu, SENSOR_CHAN_ACCEL_XYZ, acc_val);	// In m/s²
		ZSL_VECTOR_DEF(raw_acc, 3);
		raw_acc_vec[0] = sensor_value_to_float(&acc_val[0]);
		raw_acc_vec[1] = sensor_value_to_float(&acc_val[1]);
		raw_acc_vec[2] = sensor_value_to_float(&acc_val[2]);

		bool accel_clip = ZSL_ABS(raw_acc_vec[0]) >= accel_clip_limit ||
				  ZSL_ABS(raw_acc_vec[1]) >= accel_clip_limit ||
				  ZSL_ABS(raw_acc_vec[2]) >= accel_clip_limit;
		if (accel_clip)
			atomic_inc(&accel_clip_count);

		// Fetch gyro data
		struct sensor_value gyro_val[3];
		sensor_channel_get(dev_imu, SENSOR_CHAN_GYRO_XYZ, gyro_val);	// In rad/s
		ZSL_VECTOR_DEF(raw_gyr, 3);
		raw_gyr_vec[0] = sensor_value_to_float(&gyro_val[0]);
		raw_gyr_vec[1] = sensor_value_to_float(&gyro_val[1]);
		raw_gyr_vec[2] = sensor_value_to_float(&gyro_val[2]);

		bool gyro_clip = ZSL_ABS(raw_gyr_vec[0]) >= gyro_clip_limit ||
				 ZSL_ABS(raw_gyr_vec[1]) >= gyro_clip_limit ||
				 ZSL_ABS(raw_gyr_vec[2]) >= gyro_clip_limit;
		if (gyro_clip)
			atomic_inc(&gyro_clip_count);

		ZSL_VECTOR_DEF(raw_mag, 3);
		if (mag_fetched) {
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

		// TODO: Handle clipping somehow

		zsl_real_t acc_norm = zsl_vec_norm(&raw_acc);
		zsl_real_t gyr_norm = zsl_vec_norm(&raw_gyr);
		// TODO: Make use of the magnetometer
		zsl_real_t mag_norm = zsl_vec_norm(&raw_mag);

		bool acc_static = false;
		if (ZSL_ABS(acc_norm - 9.81f) < 0.5f)
			acc_static = true;

		bool gyr_static = false;
		if (gyr_norm < 0.09f)
			gyr_static = true;

		ZSL_VECTOR_DEF(accel, 3);
		zsl_vec_copy(&accel, &raw_acc);
		zsl_vec_to_unit(&accel);

		if (acc_static && gyr_static) {
			const zsl_real_t alpha = (1.0f / IMU_SAMPLING_FREQUENCY) / 2.0f;

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


		MadgwickAHRSupdateIMU(gyro_vec[0], gyro_vec[1], gyro_vec[2], accel_vec[0], accel_vec[1], accel_vec[2]);
		k_spinlock_key_t key = k_spin_lock(&cur_orient_lock);
		cur_orient_q.r = q0;
		cur_orient_q.i = q1;
		cur_orient_q.j = q2;
		cur_orient_q.k = q3;
		k_spin_unlock(&cur_orient_lock, key);


		if ((sample_idx % TELEMETRY_DECIM_FACTOR) == 0) {
			struct sensor_value temp;
			sensor_channel_get(dev_imu, SENSOR_CHAN_DIE_TEMP, &temp);
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

		int64_t t_end = k_uptime_ticks();

		if ((sample_idx % (uint32_t)IMU_SAMPLING_FREQUENCY) == 0) {
			// LOG_DBG("loop time: %lld us", k_ticks_to_us_near64(t_end - t_start));
			// LOG_DBG("gyro bias: [%f, %f, %f]", (double)gyro_bias_vec[0], (double)gyro_bias_vec[1], (double)gyro_bias_vec[2]);
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

	const float sensitivity = 1600.0f;
	const float dt_nominal = 0.01f;

	int64_t last_move_ts = 0;
	while (true) {
		k_sleep(K_MSEC(10));

		k_spinlock_key_t key = k_spin_lock(&cur_orient_lock);
		struct zsl_quat q_curr = cur_orient_q;
		k_spin_unlock(&cur_orient_lock, key);

		// Compute delta rotation:
		//  q_delta = q_curr * q_prev^(-1)
		struct zsl_quat q_prev_conj;
		zsl_quat_conj(&q_prev, &q_prev_conj);

		struct zsl_quat q_delta;
		zsl_quat_mult(&q_curr, &q_prev_conj, &q_delta);

		q_prev = q_curr;

#if 1
		struct zsl_euler e;
		zsl_quat_to_euler(&q_delta, &e);
		float raw_dx = e.z;
		float raw_dy = e.y;
#else
		// Approximation becuase `sin(x) = x` for small-ish x.
		float raw_dx = 2.0f * q_delta.k;
		float raw_dy = 2.0f * q_delta.j;
#endif

		float dt = 0.01f;
		int64_t now = k_uptime_ticks();
		if (last_move_ts != 0) {
			dt = k_ticks_to_us_near64(now - last_move_ts) / 1000000.0f;
		}
		last_move_ts = now;

		// TODO: Deadzones
		// TODO: Some filtering
		// TODO: Acceleration curve, maybe?

		float dx = -(sensitivity * (dt / dt_nominal)) * raw_dx;
		float dy =  (sensitivity * (dt / dt_nominal)) * raw_dy;

		int16_t idx = roundf(dx);
		int16_t idy = roundf(dy);

		if (idx != 0 || idy != 0)
			hm_input_report_mouse_move(idx, idy, K_FOREVER);
	}
}
K_THREAD_DEFINE(hid_mouse_tid, 2048, hid_mouse_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, 0);

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
		LOG_DBG("trigger: %u (total: %u), clip: a: %u, g: %u", trigger_delta, trigger_count, accel_clip_cnt, gyro_clip_cnt);
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
