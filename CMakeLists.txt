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

add_executable(tracker src/main.cpp src/boat_simulator.cpp src/gps_sensor.cpp)
target_link_libraries(tracker PRIVATE Eigen3::Eigen)
target_include_directories(tracker PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_options(tracker PRIVATE -Wall -Wextra -Wpedantic)
