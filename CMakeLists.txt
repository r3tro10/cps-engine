cmake_minimum_required(VERSION 3.10)
project(CPSEngine VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1. Core Library (Compiled Once)
add_library(cps_core STATIC
    src/sensor.cpp
    src/sensor_manager.cpp
    src/control/pid.cpp
    src/actuators/mock_actuator.cpp
)
target_include_directories(cps_core PUBLIC include)

# 2. Main Simulation Executable
add_executable(cps_engine src/main.cpp)
target_link_libraries(cps_engine PRIVATE cps_core)

# 3. Deterministic Unit Test Suite
enable_testing()
add_executable(test_pid tests/test_pid.cpp)
target_link_libraries(test_pid PRIVATE cps_core)
add_test(NAME PIDControlTest COMMAND test_pid)