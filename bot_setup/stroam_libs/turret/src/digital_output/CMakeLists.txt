
add_library(digitalOutput OBJECT laserPointer.cpp)
target_include_directories(digitalOutput PRIVATE .)

target_link_libraries(digitalOutput PRIVATE pigpio)