cmake_minimum_required(VERSION 3.8)
project(TestSystemComponents LANGUAGES CXX)

add_executable(testComponents main.cpp)

# Specify the path to the directory where Stroam's static library exists
set(COMPONENTS_LIBRARY_PATH "/home/redbot/projects/Foam-Ball-Turret-Bot/bot_setup/stroam_system_components/build")

# Specify the path to the include directory for Stroam's static library headers
set(COMPONENTS_LIBRARY_INCLUDE_DIR "../../stroam_system_components/include")

include(FetchContent)
FetchContent_Declare(
  libgpio
  GIT_REPOSITORY https://github.com/buildrobotsbetter/libgpio.git 
  GIT_TAG v0.2.0        
)
FetchContent_MakeAvailable(libgpio)

target_link_libraries(testComponents PRIVATE
	${COMPONENTS_LIBRARY_PATH}/libstroamSystemComponents.a
	libgpio
)

target_include_directories(testComponents PRIVATE 
	${libgpio_SOURCE_DIR}/include
	${COMPONENTS_LIBRARY_INCLUDE_DIR}
)
