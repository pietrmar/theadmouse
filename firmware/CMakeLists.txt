cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(theadmouse)

target_sources(app PRIVATE
		src/main.c
		src/hog.c
)
