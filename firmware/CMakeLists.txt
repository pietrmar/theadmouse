cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(theadmouse)

target_sources(app PRIVATE
		src/main.c

		src/at_cmd.c
		src/at_cmd_param.c
		src/ble_hid_service.c
		src/button_manager.c
		src/cmd_uart.c
		src/headmouse_input.c
		src/motion_engine.c
		src/slot_manager.c

		src/MadgwickAHRS/MadgwickAHRS.c

		src/keymaps/KeyboardLayout_da_DK.c
		src/keymaps/KeyboardLayout_de_DE.c
		src/keymaps/KeyboardLayout_en_US.c
		src/keymaps/KeyboardLayout_es_ES.c
		src/keymaps/KeyboardLayout_fr_FR.c
		src/keymaps/KeyboardLayout_hu_HU.c
		src/keymaps/KeyboardLayout_it_IT.c
		src/keymaps/KeyboardLayout_pt_BR.c
		src/keymaps/KeyboardLayout_pt_PT.c
		src/keymaps/KeyboardLayout_sv_SE.c
)

if(CONFIG_TELEMETRY)
	target_sources(app PRIVATE src/telemetry_uart.c)
endif()

if(CONFIG_LSM6DSL_CUSTOM)
	target_sources(app PRIVATE
		drivers/sensor/st/lsm6dsl/lsm6dsl.c
		drivers/sensor/st/lsm6dsl/lsm6dsl_i2c.c
		drivers/sensor/st/lsm6dsl/lsm6dsl_spi.c
	)

	if (CONFIG_LSM6DSL_TRIGGER)
		target_sources(app PRIVATE drivers/sensor/st/lsm6dsl/lsm6dsl_trigger.c)
	endif()

	if (CONFIG_LSM6DSL_SENSORHUB)
		target_sources(app PRIVATE drivers/sensor/st/lsm6dsl/lsm6dsl_trigger.c)
	endif()
endif()

if(CONFIG_LIS2MDL_CUSTOM)
	target_include_directories(app PRIVATE ${ZEPHYR_BASE}/drivers/sensor/st/stmemsc)

	target_sources(app PRIVATE drivers/sensor/st/lis2mdl/lis2mdl.c)

	if (CONFIG_LIS2MDL_TRIGGER)
		target_sources(app PRIVATE drivers/sensor/st/lis2mdl/lis2mdl_trigger.c)
	endif()
endif()
