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
include CMakeFiles/gpd_hand_geometry.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gpd_hand_geometry.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gpd_hand_geometry.dir/flags.make

CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.o: CMakeFiles/gpd_hand_geometry.dir/flags.make
CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.o: ../src/gpd/candidate/hand_geometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bglyvv/thesis_ws/src/gpd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.o -c /home/bglyvv/thesis_ws/src/gpd/src/gpd/candidate/hand_geometry.cpp

CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bglyvv/thesis_ws/src/gpd/src/gpd/candidate/hand_geometry.cpp > CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.i

CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bglyvv/thesis_ws/src/gpd/src/gpd/candidate/hand_geometry.cpp -o CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.s

# Object files for target gpd_hand_geometry
gpd_hand_geometry_OBJECTS = \
"CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.o"

# External object files for target gpd_hand_geometry
gpd_hand_geometry_EXTERNAL_OBJECTS =

libgpd_hand_geometry.a: CMakeFiles/gpd_hand_geometry.dir/src/gpd/candidate/hand_geometry.cpp.o
libgpd_hand_geometry.a: CMakeFiles/gpd_hand_geometry.dir/build.make
libgpd_hand_geometry.a: CMakeFiles/gpd_hand_geometry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bglyvv/thesis_ws/src/gpd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libgpd_hand_geometry.a"
	$(CMAKE_COMMAND) -P CMakeFiles/gpd_hand_geometry.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gpd_hand_geometry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gpd_hand_geometry.dir/build: libgpd_hand_geometry.a

.PHONY : CMakeFiles/gpd_hand_geometry.dir/build

CMakeFiles/gpd_hand_geometry.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gpd_hand_geometry.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gpd_hand_geometry.dir/clean

CMakeFiles/gpd_hand_geometry.dir/depend:
	cd /home/bglyvv/thesis_ws/src/gpd/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bglyvv/thesis_ws/src/gpd /home/bglyvv/thesis_ws/src/gpd /home/bglyvv/thesis_ws/src/gpd/build /home/bglyvv/thesis_ws/src/gpd/build /home/bglyvv/thesis_ws/src/gpd/build/CMakeFiles/gpd_hand_geometry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gpd_hand_geometry.dir/depend

