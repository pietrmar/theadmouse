#pragma once

enum app_pm_lock_reason {
	// Periodic USB reporting (enabled/disabled with AT comamnds SR/ER)
	APP_PM_LOCK_REPORT	= 0,

	APP_PM_LOCK_MAX,
};

int app_pm_init(void);
void app_pm_lock_acquire(enum app_pm_lock_reason reason);
void app_pm_lock_release(enum app_pm_lock_reason reason);
void app_pm_report_activity(void);

void app_pm_debug_execute_idle_work_handler(void);
