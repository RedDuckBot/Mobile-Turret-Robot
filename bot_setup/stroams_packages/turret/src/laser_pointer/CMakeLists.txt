add_library(laser OBJECT laserPointer.cpp)
target_include_directories(laser PUBLIC .)

#libgpio target specified by parent directory 
target_link_libraries(laser PUBLIC libgpio)