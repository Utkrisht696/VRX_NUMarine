cmake_minimum_required(VERSION 3.5)
project(guidance)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
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

ament_package()
