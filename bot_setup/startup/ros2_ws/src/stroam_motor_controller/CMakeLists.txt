cmake_minimum_required(VERSION 3.8)
project(stroam_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(stroam_interfaces REQUIRED)

#Include Stroam's libraries
include("include/TurretTargets.cmake")
include("include/DeviceTargets.cmake")
include("include/MotorDriverTargets.cmake")

add_executable(motor_controller_node src/motorControllerNode.cpp)

target_link_libraries(motor_controller_node 
	MotorDriver::motorDriver
	Turret::turret
	Device::device
	pigpiod_if2
	fmt::fmt
)

#Link ROS2 packages
ament_target_dependencies(motor_controller_node rclcpp rclcpp_action stroam_interfaces)

install(TARGETS
	motor_controller_node
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
