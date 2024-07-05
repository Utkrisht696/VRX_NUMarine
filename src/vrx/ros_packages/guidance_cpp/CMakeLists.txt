cmake_minimum_required(VERSION 3.5)
project(guidance_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Trajectory.srv"
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS guidance_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
