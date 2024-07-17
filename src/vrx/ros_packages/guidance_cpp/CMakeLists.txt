cmake_minimum_required(VERSION 3.5)
project(guidance_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include Eigen unsupported modules
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported)

add_executable(guidance_service src/guidance_service.cpp)

ament_target_dependencies(guidance_service rclcpp Eigen3)

install(TARGETS
  guidance_service
  DESTINATION lib/${PROJECT_NAME})

ament_package()
