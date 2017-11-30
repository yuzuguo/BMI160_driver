cmake_minimum_required(VERSION 3.8)
project(BMI160_driver)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        bmi160.c
        main.c)

add_executable(bmi160_demo ${SOURCE_FILES})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})