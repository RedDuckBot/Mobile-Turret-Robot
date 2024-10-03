#Add turret's components
add_subdirectory(digital_output)
add_subdirectory(servo)

#Compose turret with all it's components
add_library(turret STATIC
	turret.cpp
	$<TARGET_OBJECTS:digitalOutput>
	$<TARGET_OBJECTS:servo>
)

target_include_directories(turret PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/digital_output
	${CMAKE_CURRENT_SOURCE_DIR}/servo
)
