cmake_minimum_required(VERSION 3.14)
project(TurretSystem)

#Bring in target 'device' from Stroam's device lib
include("DeviceTargets.cmake")

add_subdirectory(src bin)
set(EXPORT_DIR "${CMAKE_CURRENT_BINARY_DIR}/cmake")

#Expose turret interface to consumers of turret library
target_include_directories(turret INTERFACE include)

export(TARGETS turret
	FILE "${EXPORT_DIR}/TurretTargets.cmake"
	NAMESPACE Turret::
)


