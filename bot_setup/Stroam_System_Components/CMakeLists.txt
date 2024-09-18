cmake_minimum_required(VERSION 3.8)
project(StroamSystemComponents)

target_include_directories(StroamSystemComponents PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/Components
	${CMAKE_CURRENT_SOURCE_DIR}/Turret
)

add_library(StroamSystemComponents STATIC 
	src/Components/Camera.cpp
	src/Components/LaserPointer.cpp
	src/Components/Servo.cpp
	src/Components/ShootingMotors.cpp
	src/Turret/Turret.cpp
)