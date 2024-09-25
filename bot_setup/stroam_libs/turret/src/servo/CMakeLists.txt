add_library(servo OBJECT servo.cpp)
target_include_directories(servo PRIVATE .)

target_link_libraries(servo PRIVATE pigpio)