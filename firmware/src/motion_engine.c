#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <zsl/orientation/fusion/madgwick.h>

LOG_MODULE_REGISTER(motion_engine, CONFIG_MOTION_ENGINE_LOG_LEVEL);

#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(mpi_imu))
static const struct device *const dev_imu = DEVICE_DT_GET(DT_CHOSEN(mpi_imu));
#else
static const struct device *const dev_imu = NULL;
#endif


// Possible values for the LSM6DSL from the datasheet:
//   1.6, 12.5, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664
#define IMU_SAMPLING_FREQUENCY	416.0f


static atomic_t imu_trigger_count = ATOMIC_INIT(0);
static atomic_t imu_fetch_count = ATOMIC_INIT(0);

static K_SEM_DEFINE(imu_trigger_sem, 0, 1);
static void imu_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	atomic_inc(&imu_trigger_count);
	k_sem_give(&imu_trigger_sem);
}

static void imu_thread(void *, void *, void *)
{
	while (true) {
		k_sem_take(&imu_trigger_sem, K_FOREVER);
		sensor_sample_fetch_chan(dev_imu, SENSOR_CHAN_ALL);
		atomic_inc(&imu_fetch_count);
	}
}
// NOTE: `imu_trigger_handler()` should run at `-2`, and the `imu_thread` then one
// prio lower at `-1`.
K_THREAD_DEFINE(imu_tid, 2048, imu_thread, NULL, NULL, NULL, K_PRIO_COOP(1), 0, 0);

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

	LOG_DBG("trigger: %u (total: %u), fetch: %u (total: %u)", trigger_delta, trigger_count, fetch_delta, fetch_count);
}
K_TIMER_DEFINE(imu_monitor_timer, imu_monitor_cb, NULL);

int motion_engine_init(void)
{
	if (dev_imu == NULL || !device_is_ready(dev_imu)) {
		LOG_ERR("Device is not ready");
		return -ENODEV;
	}

	int ret;
	struct sensor_value odr_value;
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
