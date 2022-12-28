cmake_minimum_required(VERSION 3.7)

project(SwarmSimulationTester)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

include(FetchContent)
FetchContent_Declare(
  googletest
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# enable_testing()

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

include_directories(../header)

add_executable(SwarmAgentTester
    SwarmAgentTester.cpp
    ../src/SimObjABC.cpp
    ../src/environmentManager.cpp
    ../src/swarmAgent.cpp
    ../src/sensor.cpp
    ../header/SimObjABC.hpp
    ../header/swarmAgent.hpp
    ../header/environmentManager.hpp
    ../header/sensor.hpp
)

target_link_libraries(
  SwarmAgentTester

  PRIVATE
        ${PYTHON_LIBRARIES}
        Python3::NumPy
  
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(SwarmAgentTester)
