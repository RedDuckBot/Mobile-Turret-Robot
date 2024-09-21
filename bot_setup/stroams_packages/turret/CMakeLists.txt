cmake_minimu_required(VERSION 3.8)
project(TurretSystem)

#Get a C++ library wrapper around the C library pigpio to control RPI GPIO pins
include(FetchContent)
FetchContent_Declare(
  libgpio
  GIT_REPOSITORY https://github.com/buildrobotsbetter/libgpio.git 
  GIT_TAG v0.2.0         
)
FetchCentent_MakeAvailable(libgpio)

add_subdirectory(include/turret)

# Create the final turret library 
add_library(turret
    turret.cpp                  
    $<TARGET_OBJECTS:engine>    
    $<TARGET_OBJECTS:wheel>     
    $<TARGET_OBJECTS:body>      
)

target_link_libraries(turret PUBLIC libgpio)

target_include_directories(turret PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/turret
  ${libgpio_SOURCE_DIR}/include
)