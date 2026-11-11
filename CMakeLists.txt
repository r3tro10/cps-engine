cmake_minimum_required(VERSION 3.10)
project(CPSEngine VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Locate System Dependencies
find_package(Eigen3 REQUIRED NO_MODULE)

# Core Engine Library
add_library(cps_core STATIC
    src/sensor.cpp
    src/sensor_manager.cpp
    src/control/pid.cpp
    src/actuators/mock_actuator.cpp
    src/kinematics/kinematics.cpp
)
add_executable(cps_engine src/main.cpp)
target_link_libraries(cps_engine PRIVATE cps_core)

target_include_directories(cps_core PUBLIC include)
target_link_libraries(cps_core PUBLIC Eigen3::Eigen)

# Executable & Unit Tests
enable_testing()

# Existing PID tests
add_executable(test_pid tests/test_pid.cpp)
target_link_libraries(test_pid PRIVATE cps_core)
add_test(NAME PIDControlTest COMMAND test_pid)

# Kinematics Verification Test Suite
add_executable(test_kinematics tests/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE cps_core)
add_test(NAME KinematicsTest COMMAND test_kinematics)