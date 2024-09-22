add_library(shootingMotors OBJECT shootingMotors.cpp)
target_include_directories(shootingMotors PRIVATE .) 

#target libgpio defined in src/ directory
target_link_libraries(servo PRIVATE libgpio)