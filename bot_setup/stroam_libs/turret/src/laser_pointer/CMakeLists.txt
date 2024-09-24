
add_library(laser OBJECT laserPointer.cpp)
target_include_directories(laser PRIVATE .)

target_link_libraries(laser PRIVATE pigpio)