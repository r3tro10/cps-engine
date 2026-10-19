cmake_minimum_required(VERSION 3.10)
project(CPSEngine VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(cps_engine main.cpp sensor.cpp sensor_manager.cpp src/control/pid.cpp)
target_include_directories(cps_engine PRIVATE include)