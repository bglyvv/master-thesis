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
CMAKE_SOURCE_DIR = /home/bglyvv/thesis_ws/src/gpd

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bglyvv/thesis_ws/src/gpd/build

# Include any dependencies generated for this target.
include CMakeFiles/gpd_image_1_channels_strategy.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gpd_image_1_channels_strategy.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gpd_image_1_channels_strategy.dir/flags.make

CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.o: CMakeFiles/gpd_image_1_channels_strategy.dir/flags.make
CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.o: ../src/gpd/descriptor/image_1_channels_strategy.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bglyvv/thesis_ws/src/gpd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.o -c /home/bglyvv/thesis_ws/src/gpd/src/gpd/descriptor/image_1_channels_strategy.cpp

CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bglyvv/thesis_ws/src/gpd/src/gpd/descriptor/image_1_channels_strategy.cpp > CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.i

CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bglyvv/thesis_ws/src/gpd/src/gpd/descriptor/image_1_channels_strategy.cpp -o CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.s

# Object files for target gpd_image_1_channels_strategy
gpd_image_1_channels_strategy_OBJECTS = \
"CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.o"

# External object files for target gpd_image_1_channels_strategy
gpd_image_1_channels_strategy_EXTERNAL_OBJECTS =

libgpd_image_1_channels_strategy.a: CMakeFiles/gpd_image_1_channels_strategy.dir/src/gpd/descriptor/image_1_channels_strategy.cpp.o
libgpd_image_1_channels_strategy.a: CMakeFiles/gpd_image_1_channels_strategy.dir/build.make
libgpd_image_1_channels_strategy.a: CMakeFiles/gpd_image_1_channels_strategy.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bglyvv/thesis_ws/src/gpd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libgpd_image_1_channels_strategy.a"
	$(CMAKE_COMMAND) -P CMakeFiles/gpd_image_1_channels_strategy.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gpd_image_1_channels_strategy.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gpd_image_1_channels_strategy.dir/build: libgpd_image_1_channels_strategy.a

.PHONY : CMakeFiles/gpd_image_1_channels_strategy.dir/build

CMakeFiles/gpd_image_1_channels_strategy.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gpd_image_1_channels_strategy.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gpd_image_1_channels_strategy.dir/clean

CMakeFiles/gpd_image_1_channels_strategy.dir/depend:
	cd /home/bglyvv/thesis_ws/src/gpd/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bglyvv/thesis_ws/src/gpd /home/bglyvv/thesis_ws/src/gpd /home/bglyvv/thesis_ws/src/gpd/build /home/bglyvv/thesis_ws/src/gpd/build /home/bglyvv/thesis_ws/src/gpd/build/CMakeFiles/gpd_image_1_channels_strategy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gpd_image_1_channels_strategy.dir/depend

