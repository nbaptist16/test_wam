# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack/build

# Include any dependencies generated for this target.
include CMakeFiles/ros_wamtrack_plugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ros_wamtrack_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ros_wamtrack_plugin.dir/flags.make

CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.o: CMakeFiles/ros_wamtrack_plugin.dir/flags.make
CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.o: ../plugins/ros_wamtrack_plugin.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.o -c /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack/plugins/ros_wamtrack_plugin.cc

CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack/plugins/ros_wamtrack_plugin.cc > CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.i

CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack/plugins/ros_wamtrack_plugin.cc -o CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.s

# Object files for target ros_wamtrack_plugin
ros_wamtrack_plugin_OBJECTS = \
"CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.o"

# External object files for target ros_wamtrack_plugin
ros_wamtrack_plugin_EXTERNAL_OBJECTS =

libros_wamtrack_plugin.so: CMakeFiles/ros_wamtrack_plugin.dir/plugins/ros_wamtrack_plugin.cc.o
libros_wamtrack_plugin.so: CMakeFiles/ros_wamtrack_plugin.dir/build.make
libros_wamtrack_plugin.so: CMakeFiles/ros_wamtrack_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libros_wamtrack_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ros_wamtrack_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ros_wamtrack_plugin.dir/build: libros_wamtrack_plugin.so

.PHONY : CMakeFiles/ros_wamtrack_plugin.dir/build

CMakeFiles/ros_wamtrack_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ros_wamtrack_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ros_wamtrack_plugin.dir/clean

CMakeFiles/ros_wamtrack_plugin.dir/depend:
	cd /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack/build /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack/build /home/nckmlb/test_wam/src/Gazebo_WAM/wamtrack/build/CMakeFiles/ros_wamtrack_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ros_wamtrack_plugin.dir/depend

