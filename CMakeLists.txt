cmake_minimum_required(VERSION 3.7)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(SwarmSimulation)

include_directories(header)

add_executable(SwarmSimulation 
    src/main.cpp
    src/swarmAgent.cpp
    src/sensor.cpp
    src/environmentManager.cpp
    header/swarmAgent.hpp
    header/sensor.hpp 
    header/environmentManager.hpp
)
