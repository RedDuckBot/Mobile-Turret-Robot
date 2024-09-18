cmake_minimum_required(VERSION 3.8)
project(StroamSystemComponents)

target_include_directories(StroamSystemComponents PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/Components
	${CMAKE_CURRENT_SOURCE_DIR}/Turret
)

#Fetching a repo that contains c/c++ libraries to control RPI GPIO pins 
include(FetchContent)
FetchContent_Declare(
	GPIOControl
	Git_REPOSITORY https://github.com/buildrobotsbetter/rpi4b_gpio-example.git
	GIT_TAG 1.3.1
)

FetchContent_MakeAvailable(GPIOControl)

#From GPIOControl only one library called libgpio will be utilized in Stroams
#system components
add_subdirectory(${GPIOControl_SOURCE_DIR}/libgpio)

add_library(StroamSystemComponents STATIC 
	src/Components/Camera.cpp
	src/Components/LaserPointer.cpp
	src/Components/Servo.cpp
	src/Components/ShootingMotors.cpp
	src/Turret/Turret.cpp
)

target_link_libraries(StroamSystemComponents PRIVATE libgpio)