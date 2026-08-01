cmake_minimum_required(VERSION 3.10)
project(CPSEngine VERSION 1.0)

# Enforce modern C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define the executable and the source files that build it
add_executable(cps_engine main.cpp sensor.cpp)