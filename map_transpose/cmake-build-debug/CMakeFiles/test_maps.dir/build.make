# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/zzh/clion-2019.2.5/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zzh/clion-2019.2.5/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zzh/maplab_ws2/src/map_transpose

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zzh/maplab_ws2/src/map_transpose/cmake-build-debug

# Utility rule file for test_maps.

# Include the progress variables for this target.
include CMakeFiles/test_maps.dir/progress.make

test_maps: CMakeFiles/test_maps.dir/build.make
	rm -rf test_maps && cp -r /home/zzh/maplab_ws2/src/maplab/tools/maplab_test_data/test_maps .
.PHONY : test_maps

# Rule to build all files generated by this target.
CMakeFiles/test_maps.dir/build: test_maps

.PHONY : CMakeFiles/test_maps.dir/build

CMakeFiles/test_maps.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_maps.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_maps.dir/clean

CMakeFiles/test_maps.dir/depend:
	cd /home/zzh/maplab_ws2/src/map_transpose/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zzh/maplab_ws2/src/map_transpose /home/zzh/maplab_ws2/src/map_transpose /home/zzh/maplab_ws2/src/map_transpose/cmake-build-debug /home/zzh/maplab_ws2/src/map_transpose/cmake-build-debug /home/zzh/maplab_ws2/src/map_transpose/cmake-build-debug/CMakeFiles/test_maps.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_maps.dir/depend

