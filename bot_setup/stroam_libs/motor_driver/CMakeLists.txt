cmake_minimum_required(VERSION 3.14)
project(Motor_Driver)

add_subdirectory(src bin)
set(EXPORT_DIR "${CMAKE_CURRENT_BINARY_DIR}/cmake")

target_include_directories(motorDriver PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)

export(TARGETS motorDriver
	FILE "${EXPORT_DIR}/MotorDriverTargets.cmake"
	NAMESPACE MotorDriver::
)