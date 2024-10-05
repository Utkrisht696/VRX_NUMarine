cmake_minimum_required(VERSION 3.5)
project(Heartbeat_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)

include_directories(include)

add_executable(heartbeat_subscriber src/heartbeat_subscriber.cpp)
ament_target_dependencies(heartbeat_subscriber rclcpp std_msgs can_msgs)

add_executable(heartbeat_publisher src/heartbeat_publisher.cpp)
ament_target_dependencies(heartbeat_publisher rclcpp std_msgs can_msgs)

install(TARGETS
  heartbeat_subscriber
  heartbeat_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()

