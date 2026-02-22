cmake_minimum_required(VERSION 3.8)
project(ros2_lifecycle)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(lifecycle_sensor src/lifecycle_sensor.cpp)

ament_target_dependencies(lifecycle_sensor
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

install(TARGETS
  lifecycle_sensor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
