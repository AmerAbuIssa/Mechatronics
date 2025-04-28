# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.1)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (laser_data LANGUAGES CXX VERSION 3.0.1)

# This is added to c++171 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## Include the location of pfms headers
find_package(pfms2ros 3.0.8 REQUIRED)

# Executables
add_executable(receive_laser receive_laser.cpp)
add_executable(receive_sonar receive_sonar.cpp)
add_executable(command_quadcopter command_quadcopter.cpp)

# Link the library
target_link_libraries(receive_laser pfmsconnector)
target_link_libraries(receive_sonar pfmsconnector)
target_link_libraries(command_quadcopter pfmsconnector)
