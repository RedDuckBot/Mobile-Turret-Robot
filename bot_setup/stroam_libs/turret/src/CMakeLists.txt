#Add turret's components
add_subdirectory(digital_output_device)
add_subdirectory(servo)

#Compose turret with all it's components
add_library(turret STATIC
	turret.cpp
	$<TARGET_OBJECTS:digitalOutputDevice>
	$<TARGET_OBJECTS:servo>
)

target_include_directories(turret PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/digital_output_device
	${CMAKE_CURRENT_SOURCE_DIR}/servo
)
