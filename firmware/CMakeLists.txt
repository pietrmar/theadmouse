cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(theadmouse)

target_sources(app PRIVATE
		src/main.c

		src/at_cmd.c
		src/at_cmd_param.c
		src/button_manager.c
		src/cmd_uart.c
		src/headmouse_input.c
		src/hog.c
		src/slot_manager.c
		src/telemetry_uart.c
)
