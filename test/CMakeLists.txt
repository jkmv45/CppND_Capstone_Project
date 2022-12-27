cmake_minimum_required(VERSION 3.7)

add_definitions(-std=c++17)

include(FetchContent)
FetchContent_Declare(
  googletest
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

# enable_testing()

project(SwarmSimulationTester)

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
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(SwarmAgentTester)
