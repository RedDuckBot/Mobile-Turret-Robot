cmake_minimum_required(VERSION 3.14)
project(TestTurretLibrary LANGUAGES CXX)

find_package(fmt)
include("TurretTargets.cmake")
include("DeviceTargets.cmake")

add_executable(testTurretLib main.cpp)
target_link_libraries(testTurretLib PRIVATE 
	Turret::turret 
	pigpio 
	fmt::fmt
	Device::device
)

