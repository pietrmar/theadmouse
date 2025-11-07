cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(theadmouse)

target_sources(app PRIVATE
		src/hog.c
		src/main.c
		src/at_cmd.c
		src/cmd_uart.c
		src/telemetry_uart.c

		src/MadgwickAHRS/MadgwickAHRS.c
)
