cmake_minimum_required(VERSION 3.14)
project(TestTurretLibrary LANGUAGES CXX)

include("TurretTargets.cmake")

add_executable(testTurretLib main.cpp)
target_link_libraries(testTurretLib PRIVATE Turret::turret)

