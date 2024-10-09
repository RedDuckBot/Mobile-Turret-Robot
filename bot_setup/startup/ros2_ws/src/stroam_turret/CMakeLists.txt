cmake_minimum_required(VERSION 3.8)
project(stroam_turret)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(fmt REQUIRED)
find_package(stroam_interfaces REQUIRED)

#Include Stroam's libraries
include("include/TurretTargets.cmake")
include("include/DeviceTargets.cmake")

add_executable(turret_node src/turretNode.cpp)

target_link_libraries(turret_node 
  Turret::turret
  Device::device
  pigpiod_if2
  fmt::fmt
)

#Link ROS2 packages
ament_target_dependencies(turret_node rclcpp rclcpp_action stroam_interfaces)

install(TARGETS
  turret_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
