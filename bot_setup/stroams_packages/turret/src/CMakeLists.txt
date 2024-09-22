#Add turrets sub-components
add_subdirectory(laser_pointer)
add_subdirectory(servo)
add_subdirectory(shooting_motors)

#Each sub-component depends on a C++ library wrapper of pigpio on GitHub 
include(FetchContent)
FetchContent_Declare(
	libgpio	
	GIT_REPOSITORY https://github.com/buildrobotsbetter/libgpio.git 
	GIT_TAG v0.2.0         
)
FetchContent_MakeAvailable(libgpio)

add_library(turret
	$<TARGET_OBJECTS:laser>
	$<TARGET_OBJECTS:shootingMotors>
	$<TARGET_OBJECTS:servo>
	turret.cpp
)

target_libraries_directories(turret PRIVATE libgpio)