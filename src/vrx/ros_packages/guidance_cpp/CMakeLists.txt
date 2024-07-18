cmake_minimum_required(VERSION 3.5)
project(guidance_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(guidance_service src/guidance_service.cpp)

ament_target_dependencies(guidance_service rclcpp std_msgs Eigen3)

install(TARGETS
  guidance_service
  DESTINATION lib/${PROJECT_NAME})

ament_package()
