cmake_minimum_required(VERSION 3.14)
project(Device)

add_subdirectory(src bin)
set(EXPORT_DIR "${CMAKE_CURRENT_BINARY_DIR}/cmake")

target_include_directories(device PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)

export(TARGETS device
	FILE "${EXPORT_DIR}/DeviceTargets.cmake"
	NAMESPACE Device::
)