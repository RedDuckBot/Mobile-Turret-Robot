cmake_minimum_required(VERSION 3.14)
project(TestTurretLibrary LANGUAGES CXX)

find_package(fmt REQUIRED)
include("TurretTargets.cmake")
include("DeviceTargets.cmake")

add_executable(testTurretLib main.cpp)

#Recall the ordering for linking targets matters so that the tables used
#in linking process get filled in appropriately
target_link_libraries(testTurretLib PRIVATE 
	Turret::turret 
	Device::device
	pigpio
	pigpiod_if2
	fmt::fmt
)

