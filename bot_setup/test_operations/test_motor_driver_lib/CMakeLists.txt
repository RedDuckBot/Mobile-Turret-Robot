cmake_minimum_required(VERSION 3.14)
project(TestMotorDriverLibrary LANGUAGES CXX)

find_package(fmt REQUIRED)
include("DeviceTargets.cmake")
include("MotorDriverTargets.cmake")
include("TurretTargets.cmake")

add_executable(testMotorDriverLib main.cpp)

#Recall the ordering for linking targets matters so that the tables used
#in linking process get filled in appropriately
target_link_libraries(testMotorDriverLib PRIVATE
	MotorDriver::motorDriver
	Turret::turret
	Device::device
	pigpiod_if2
	fmt::fmt
)