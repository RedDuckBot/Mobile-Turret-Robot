add_library(digitalOutput OBJECT digitalOutputDevice.cpp)
target_link_libraries(digitalOutput PUBLIC pigpio_if2 Device::device)