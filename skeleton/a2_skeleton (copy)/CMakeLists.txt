# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.0)

# Change project name
project (a2_skeleton)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

set(BUILD_TESTS ON)

find_package(Threads)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(pfms2ros 3.0.7 REQUIRED)

# We supply a logger library to read/write Point3d to a file
add_library(logger logger.cpp)

add_library(controller STATIC controller.cpp ackerman.cpp quadcopter.cpp)
add_library(mission STATIC mission.cpp)
target_link_libraries(controller pfmsconnector audi Eigen3::Eigen)
target_link_libraries(mission pfmsconnector)

# Create executable with  cpp files needed
add_executable(a2 main.cpp)

# Cmake will link against the libraries in final step
target_link_libraries(a2 logger controller mission pfmsconnector)

if(BUILD_TESTS)
   add_subdirectory(test)
endif()
