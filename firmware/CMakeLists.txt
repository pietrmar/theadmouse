cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
	${CMAKE_CURRENT_SOURCE_DIR}/drivers/sensor/st/lsm6dsl
)

find_package(Zephyr)
project(theadmouse)

target_sources(app PRIVATE
		src/main.c
		src/hog.c
		src/MadgwickAHRS/MadgwickAHRS.c
)
