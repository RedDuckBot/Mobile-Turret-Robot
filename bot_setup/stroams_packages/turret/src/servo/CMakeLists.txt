add_library(servo OBJECT servo.cpp)
target_include_directories(servo PUBLIC .)
target_link_libraries(servo PUBLIC libgpio)