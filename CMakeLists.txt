cmake_minimum_required(VERSION 3.10)
project(CPSEngine VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Main Engine Executable
add_executable(cps_engine
    src/main.cpp
    src/sensor.cpp
    src/sensor_manager.cpp
    src/control/pid.cpp
    src/actuators/mock_actuator.cpp
)
target_include_directories(cps_engine PRIVATE include)

# Unit Test Suite
enable_testing()
add_executable(test_pid
    tests/test_pid.cpp
    src/control/pid.cpp
    src/actuators/mock_actuator.cpp
)
target_include_directories(test_pid PRIVATE include)
add_test(NAME PIDControlTest COMMAND test_pid)