# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0)

# Any ppoject name will suffice, this has connotaions when using advanced CMake Features
project (week4)

# Let's create a library
# Create the library, replace <library_name> with the name of library
# Replace <cpp_files> with the files that form part of library
# add_library(<library_name> SHARED <cpp_files>)
add_library(shapes SHARED circle.cpp shape.cpp rectangle.cpp triangle.cpp)

# We have another library
add_library(shapeprocessing SHARED shapeprocessing.cpp)
# This library depends on shapes library 
target_link_libraries(shapeprocessing shapes)

# We can now also create an executable, though this is not shared with others (just the library)
# So we can even comment this out.
add_executable(battleship main.cpp)

# We now have a library and an executable, we need to link each other
target_link_libraries(battleship shapeprocessing)

#################################################################
# UNIT_TESTING SECTION
#
# We add the test directory and enable testing
add_subdirectory(test)

#################################################################
# CREATE LIBRARY SECTION
#
# If we want to install the library, what do we do now.

# If you have sudo access on your computer the install process will install your library to default install locations
# Which on a Linux system is /usr/local/lib and /usr/local/include

# Alternative, you can specify the install be any directory of choise
# 1. Below installs to your build directory
#set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})
# 2. This would install to your home folder and within subfolder of project_name (ie $HOME/shapes)
#set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/${PROJECT_NAME}")

#install(TARGETS <library_name> DESTINATION lib PUBLIC_HEADER DESTINATION include)
#install(TARGETS shapes DESTINATION lib PUBLIC_HEADER DESTINATION include)
