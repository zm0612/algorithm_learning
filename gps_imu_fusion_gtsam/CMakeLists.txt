cmake_minimum_required(VERSION 3.10)
project(gps_imu_fusion_gtsam)

set(CMAKE_CXX_FLAGS "-Wall -O3")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

add_executable(gps_imu_fusion_gtsam main.cpp pre_integration_data_preparer.cpp)
target_link_libraries(gps_imu_fusion_gtsam gtsam)
