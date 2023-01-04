cmake_minimum_required(VERSION 3.7)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(SwarmSimulation)

enable_testing()

# add_subdirectory(src)
add_subdirectory(test)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# find python libraries
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

# populate matplotlib repository
include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})



# Main Project Setup
include_directories(header)

add_executable(SwarmSimulation 
    src/main.cpp
    src/SimObjABC.cpp
    src/swarmAgent.cpp
    src/sensor.cpp
    src/environmentManager.cpp
    header/SimObjABC.hpp
    header/swarmAgent.hpp
    header/sensor.hpp 
    header/environmentManager.hpp
)

# link python and numpy
target_link_libraries(SwarmSimulation
    PRIVATE
        ${PYTHON_LIBRARIES}
        Python3::NumPy
)