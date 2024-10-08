cmake_minimum_required(VERSION 3.8)
project(stroam_turret)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

#Include Stroam's libraries
include("include/TurretTargets.cmake")
include("include/DeviceTargets.cmake")

add_executable(turret_lifecycle_node src/turretNode.cpp)

target_link_libraries(turret_lifecycle_node PRIVATE
  Turret::turret
  Device::device
  pigpiod_if2
  fmt::fmt
)

ament_target_dependencies(turret_lifecycle_node rclcpp rclcpp_lifecycle)

install(TARGETS
  turret_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
