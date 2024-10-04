add_library(servo OBJECT servo.cpp)
target_link_libraries(servo PRIVATE pigpio_if2 Device::device)