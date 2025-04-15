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
		src/slot_manager.c
		src/telemetry_uart.c
)

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
