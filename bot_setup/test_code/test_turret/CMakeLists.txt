cmake_minimum_required(VERSION 3.22)
project(TestTurret LANGUAGES CXX)

set(CMAKE_XCC_STANDARD 23)
set(CMAKE_XCC_STANDARD_REQURED ON)

find_package(fmt REQUIRED)

add_executable(testTurret main.cpp)
target_link_libraries(testTurret PRIVATE fmt::fmt pigpio)