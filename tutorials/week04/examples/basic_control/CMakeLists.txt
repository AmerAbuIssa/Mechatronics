# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.1)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (basic_control LANGUAGES CXX VERSION 3.0.1)

# This is added to c++171 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## Include the location of pfms headers
find_package(pfms2ros 3.0.7 REQUIRED)

# Executables
add_executable(control_ackerman control_ackerman.cpp)

# Link the library
target_link_libraries(control_ackerman pfmsconnector audi)

