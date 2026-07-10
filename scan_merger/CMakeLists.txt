cmake_minimum_required(VERSION 3.20)
project(scan_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(laser_geometry REQUIRED)

add_executable(scan_merger
  src/scan_merger.cpp
)

ament_target_dependencies(scan_merger
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  message_filters
  laser_geometry
)

install(
  TARGETS scan_merger
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()