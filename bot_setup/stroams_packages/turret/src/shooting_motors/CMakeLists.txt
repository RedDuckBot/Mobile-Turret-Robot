add_library(shootingMotors OBJECT shootingMotors.cpp)
target_include_directories(shootingMotors PUBLIC .) 
target_link_libraries(servo PUBLIC libgpio)