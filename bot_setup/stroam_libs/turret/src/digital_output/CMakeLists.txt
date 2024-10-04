add_library(digitalOutput OBJECT digitalOutputDevice.cpp)
target_link_libraries(digitalOutput PRIVATE pigpio_if2 Device::device)