#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app_power_manager.h"

#include "motion_engine.h"

LOG_MODULE_REGISTER(app_power_manager, CONFIG_APP_POWER_MANAGER_LOG_LEVEL);

static const k_timeout_t app_pm_idle_timeout = K_SECONDS(30);

static struct k_spinlock app_pm_spinlock;
static uint32_t app_pm_lock_mask = 0;

// This is the idle state of the power manager
static bool app_pm_target_idle = false;

// The actual idle state of the app components, the `app_pm_idle_state_sync_work_handler`
// will take `app_pm_target_idle`, resume/suspend the componenets and then updated
// `app_pm_actual_idle` accordingly.
static bool app_pm_actual_idle = false;

// TODO: Currently the consumers are hardcoded here on suspend/resume, we probably should
// implement something more nicer where consumers can register suspend/resume callbacks.
static void app_pm_suspend_app_componenets(void)
{
	LOG_INF("Suspending app components");
	int ret = motion_engine_suspend();
	if (ret < 0) {
		LOG_ERR("Failed to suspend motion engine: %d", ret);
	}
}

static void app_pm_resume_app_componenets(void)
{
	LOG_INF("Resuming app components");
	int ret = motion_engine_resume();
	if (ret < 0) {
		LOG_ERR("Failed to resume motion engine: %d", ret);
	}
}

static void app_pm_idle_state_sync_work_handler(struct k_work *work)
{
	do {
		k_spinlock_key_t key = k_spin_lock(&app_pm_spinlock);
		bool target_idle = app_pm_target_idle;

		if (target_idle == app_pm_actual_idle) {
			// We are already at the requested state, exit the loop
			k_spin_unlock(&app_pm_spinlock, key);
			break;
		}
		k_spin_unlock(&app_pm_spinlock, key);

		if (target_idle) {
			// TODO: Add logging here
			app_pm_suspend_app_componenets();
		} else {
			// TODO: Add logging here
			app_pm_resume_app_componenets();
		}

		// Update the actual idle state after the componenets have been suspended/resumed
		key = k_spin_lock(&app_pm_spinlock);
		app_pm_actual_idle = target_idle;
		k_spin_unlock(&app_pm_spinlock, key);

		// In the meantime `app_pm_target_idle` might have changed so
		// we run another iteration of the check from above.
	} while (1);
}
static K_WORK_DEFINE(app_pm_idle_state_sync_work, app_pm_idle_state_sync_work_handler);

static void app_pm_idle_work_handler(struct k_work *work)
{
	LOG_DBG("Enter");

	k_spinlock_key_t key = k_spin_lock(&app_pm_spinlock);

	if (app_pm_lock_mask == 0 && !app_pm_target_idle) {
		app_pm_target_idle = true;
		k_work_submit(&app_pm_idle_state_sync_work);
	}

	k_spin_unlock(&app_pm_spinlock, key);
}
static K_WORK_DELAYABLE_DEFINE(app_pm_idle_work, app_pm_idle_work_handler);

void app_pm_debug_execute_idle_work_handler(void)
{
	app_pm_idle_work_handler(NULL);
}

void app_pm_lock_acquire(enum app_pm_lock_reason reason)
{
	k_spinlock_key_t key = k_spin_lock(&app_pm_spinlock);

	k_work_cancel_delayable(&app_pm_idle_work);

	app_pm_lock_mask |= BIT(reason);

	if (app_pm_target_idle) {
		app_pm_target_idle = false;
		k_work_submit(&app_pm_idle_state_sync_work);
	}

	k_spin_unlock(&app_pm_spinlock, key);
}

void app_pm_lock_release(enum app_pm_lock_reason reason)
{
	k_spinlock_key_t key = k_spin_lock(&app_pm_spinlock);

	app_pm_lock_mask &= ~BIT(reason);

	if (app_pm_lock_mask == 0) {
		k_work_reschedule(&app_pm_idle_work, app_pm_idle_timeout);
	}

	k_spin_unlock(&app_pm_spinlock, key);
}

void app_pm_report_activity(void)
{
	k_spinlock_key_t key = k_spin_lock(&app_pm_spinlock);

	if (app_pm_target_idle) {
		app_pm_target_idle = false;
		k_work_submit(&app_pm_idle_state_sync_work);
	}

	if (app_pm_lock_mask == 0) {
		k_work_reschedule(&app_pm_idle_work, app_pm_idle_timeout);
	}

	k_spin_unlock(&app_pm_spinlock, key);
}

int app_pm_init(void)
{
	// Schedule the idle timer initially
	k_work_reschedule(&app_pm_idle_work, app_pm_idle_timeout);

	LOG_INF("App power-management init done");
	return 0;
}
