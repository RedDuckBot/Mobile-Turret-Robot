#Each component depends on a C++ library wrapper of pigpio on GitHub 
include(FetchContent)
FetchContent_Declare(
	libgpio	
	GIT_REPOSITORY https://github.com/buildrobotsbetter/libgpio.git 
	GIT_TAG v0.2.0         
)
FetchContent_MakeAvailable(libgpio)

#Add turret's components
add_subdirectory(laser_pointer)
add_subdirectory(servo)
add_subdirectory(shooting_motors)

#Compose turret with all it's components
add_library(turret STATIC
	turret.cpp
	$<TARGET_OBJECTS:laser>
	$<TARGET_OBJECTS:shootingMotors>
	$<TARGET_OBJECTS:servo>
)

target_include_directories(turret PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/laser_pointer
	${CMAKE_CURRENT_SOURCE_DIR}/servo
	${CMAKE_CURRENT_SOURCE_DIR}/shooting_motors
)
