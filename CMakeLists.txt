cmake_minimum_required(VERSION 3.5)
project(livox_mapping)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

include_directories(include ${PROJECT_NAME}_INTERFACES_INCLUDE_DIRS)

add_executable(livox_mapping_case src/livox_mapping_case.cpp)
ament_target_dependencies(livox_mapping_case
  rclcpp
  pcl_conversions
  pcl_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  livox_interfaces
)

install(TARGETS
  livox_mapping_case
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz_cfg/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  msg/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
