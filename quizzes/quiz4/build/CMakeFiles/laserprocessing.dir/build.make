# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/a

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/build

# Include any dependencies generated for this target.
include CMakeFiles/laserprocessing.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/laserprocessing.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/laserprocessing.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/laserprocessing.dir/flags.make

CMakeFiles/laserprocessing.dir/laserprocessing.cpp.o: CMakeFiles/laserprocessing.dir/flags.make
CMakeFiles/laserprocessing.dir/laserprocessing.cpp.o: /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/a/laserprocessing.cpp
CMakeFiles/laserprocessing.dir/laserprocessing.cpp.o: CMakeFiles/laserprocessing.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/laserprocessing.dir/laserprocessing.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/laserprocessing.dir/laserprocessing.cpp.o -MF CMakeFiles/laserprocessing.dir/laserprocessing.cpp.o.d -o CMakeFiles/laserprocessing.dir/laserprocessing.cpp.o -c /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/a/laserprocessing.cpp

CMakeFiles/laserprocessing.dir/laserprocessing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/laserprocessing.dir/laserprocessing.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/a/laserprocessing.cpp > CMakeFiles/laserprocessing.dir/laserprocessing.cpp.i

CMakeFiles/laserprocessing.dir/laserprocessing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/laserprocessing.dir/laserprocessing.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/a/laserprocessing.cpp -o CMakeFiles/laserprocessing.dir/laserprocessing.cpp.s

# Object files for target laserprocessing
laserprocessing_OBJECTS = \
"CMakeFiles/laserprocessing.dir/laserprocessing.cpp.o"

# External object files for target laserprocessing
laserprocessing_EXTERNAL_OBJECTS =

liblaserprocessing.a: CMakeFiles/laserprocessing.dir/laserprocessing.cpp.o
liblaserprocessing.a: CMakeFiles/laserprocessing.dir/build.make
liblaserprocessing.a: CMakeFiles/laserprocessing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liblaserprocessing.a"
	$(CMAKE_COMMAND) -P CMakeFiles/laserprocessing.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/laserprocessing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/laserprocessing.dir/build: liblaserprocessing.a
.PHONY : CMakeFiles/laserprocessing.dir/build

CMakeFiles/laserprocessing.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/laserprocessing.dir/cmake_clean.cmake
.PHONY : CMakeFiles/laserprocessing.dir/clean

CMakeFiles/laserprocessing.dir/depend:
	cd /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/a /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/a /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/build /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/build /home/student/git/pfms-2025a-AmerAbuIssa/quizzes/quiz4/build/CMakeFiles/laserprocessing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/laserprocessing.dir/depend

