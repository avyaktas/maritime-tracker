cmake_minimum_required(VERSION 3.16)
project(maritime_tracker CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.5.2
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(Catch2)

# Shared library target: everything except main()
add_library(tracker_lib
  src/boat_simulator.cpp
  src/gps_sensor.cpp
  src/kalman_filter.cpp
  )

target_include_directories(tracker_lib PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(tracker_lib PUBLIC Eigen3::Eigen)
target_compile_options(tracker_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(tracker src/main.cpp)
target_link_libraries(tracker PRIVATE tracker_lib)
target_compile_options(tracker PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(tests tests/test_kalman.cpp)
target_link_libraries(tests PRIVATE tracker_lib Catch2::Catch2WithMain)
add_test(NAME kalman_tests COMMAND tests)
