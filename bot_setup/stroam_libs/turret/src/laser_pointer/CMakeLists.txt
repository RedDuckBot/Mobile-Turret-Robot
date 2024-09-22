add_library(laser OBJECT laserPointer.cpp)
target_include_directories(laser PRIVATE .)

#libgpio target specified by src/ directory 
target_link_libraries(laser PRIVATE libgpio)