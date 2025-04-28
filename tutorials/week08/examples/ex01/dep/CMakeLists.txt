# Find the library and files belonging to Controller Library (which is provided in the dep subdirectory)
set(CONTROLLER_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${CONTROLLER_LIB_HOME}/include)
link_directories(${CONTROLLER_LIB_HOME}/lib/)
