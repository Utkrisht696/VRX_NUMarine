cmake_minimum_required(VERSION 3.8)
project(my_can_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(kvaser_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the original executable
add_executable(can_float_publisher src/can_float_publisher.cpp)
ament_target_dependencies(can_float_publisher rclcpp can_msgs kvaser_interface geometry_msgs)

# Add the new can_thrust_pub executable
add_executable(can_thrust_pub src/can_thrust_pub.cpp)
ament_target_dependencies(can_thrust_pub rclcpp can_msgs geometry_msgs)

target_include_directories(can_float_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(can_float_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(can_thrust_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(can_thrust_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS can_float_publisher can_thrust_pub
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
