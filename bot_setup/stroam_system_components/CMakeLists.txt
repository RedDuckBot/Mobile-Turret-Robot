cmake_minimum_required(VERSION 3.8)
project(StroamSystemComponents)

#Get a C++ library wrapper around the C library pigpio to control RPI GPIO pins
include(FetchContent)
FetchContent_Declare(
  libgpio
  GIT_REPOSITORY https://github.com/buildrobotsbetter/libgpio.git 
  GIT_TAG v0.2.0         
)
FetchContent_MakeAvailable(libgpio)

include(${CMAKE_CURRENT_LIST_DIR}/sources.cmake)

add_library(stroamSystemComponents STATIC ${SOURCES})
target_link_libraries(stroamSystemComponents PUBLIC libgpio)

target_include_directories(stroamSystemComponents PUBLIC 
	${CMAKE_SOURCE_DIR}/include
	${libgpio_SOURCE_DIR}/include
)

