# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lar2019/Desktop/AppliedRoboticsStudentInterface

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lar2019/Desktop/AppliedRoboticsStudentInterface/src

# Include any dependencies generated for this target.
include CMakeFiles/clipper.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/clipper.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/clipper.dir/flags.make

CMakeFiles/clipper.dir/clipper.cpp.o: CMakeFiles/clipper.dir/flags.make
CMakeFiles/clipper.dir/clipper.cpp.o: clipper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lar2019/Desktop/AppliedRoboticsStudentInterface/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/clipper.dir/clipper.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/clipper.dir/clipper.cpp.o -c /home/lar2019/Desktop/AppliedRoboticsStudentInterface/src/clipper.cpp

CMakeFiles/clipper.dir/clipper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clipper.dir/clipper.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lar2019/Desktop/AppliedRoboticsStudentInterface/src/clipper.cpp > CMakeFiles/clipper.dir/clipper.cpp.i

CMakeFiles/clipper.dir/clipper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clipper.dir/clipper.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lar2019/Desktop/AppliedRoboticsStudentInterface/src/clipper.cpp -o CMakeFiles/clipper.dir/clipper.cpp.s

CMakeFiles/clipper.dir/clipper.cpp.o.requires:

.PHONY : CMakeFiles/clipper.dir/clipper.cpp.o.requires

CMakeFiles/clipper.dir/clipper.cpp.o.provides: CMakeFiles/clipper.dir/clipper.cpp.o.requires
	$(MAKE) -f CMakeFiles/clipper.dir/build.make CMakeFiles/clipper.dir/clipper.cpp.o.provides.build
.PHONY : CMakeFiles/clipper.dir/clipper.cpp.o.provides

CMakeFiles/clipper.dir/clipper.cpp.o.provides.build: CMakeFiles/clipper.dir/clipper.cpp.o


# Object files for target clipper
clipper_OBJECTS = \
"CMakeFiles/clipper.dir/clipper.cpp.o"

# External object files for target clipper
clipper_EXTERNAL_OBJECTS =

libclipper.so: CMakeFiles/clipper.dir/clipper.cpp.o
libclipper.so: CMakeFiles/clipper.dir/build.make
libclipper.so: CMakeFiles/clipper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lar2019/Desktop/AppliedRoboticsStudentInterface/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libclipper.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/clipper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/clipper.dir/build: libclipper.so

.PHONY : CMakeFiles/clipper.dir/build

CMakeFiles/clipper.dir/requires: CMakeFiles/clipper.dir/clipper.cpp.o.requires

.PHONY : CMakeFiles/clipper.dir/requires

CMakeFiles/clipper.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clipper.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clipper.dir/clean

CMakeFiles/clipper.dir/depend:
	cd /home/lar2019/Desktop/AppliedRoboticsStudentInterface/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lar2019/Desktop/AppliedRoboticsStudentInterface /home/lar2019/Desktop/AppliedRoboticsStudentInterface /home/lar2019/Desktop/AppliedRoboticsStudentInterface/src /home/lar2019/Desktop/AppliedRoboticsStudentInterface/src /home/lar2019/Desktop/AppliedRoboticsStudentInterface/src/CMakeFiles/clipper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clipper.dir/depend

