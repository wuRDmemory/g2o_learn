# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /home/ubuntu/Downloads/clion-2017.3.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/ubuntu/Downloads/clion-2017.3.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/Project/CLionProject/G2O/demo2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/Project/CLionProject/G2O/demo2/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/demo2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/demo2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/demo2.dir/flags.make

CMakeFiles/demo2.dir/main.cpp.o: CMakeFiles/demo2.dir/flags.make
CMakeFiles/demo2.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/Project/CLionProject/G2O/demo2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/demo2.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo2.dir/main.cpp.o -c /home/ubuntu/Project/CLionProject/G2O/demo2/main.cpp

CMakeFiles/demo2.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo2.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/Project/CLionProject/G2O/demo2/main.cpp > CMakeFiles/demo2.dir/main.cpp.i

CMakeFiles/demo2.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo2.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/Project/CLionProject/G2O/demo2/main.cpp -o CMakeFiles/demo2.dir/main.cpp.s

CMakeFiles/demo2.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/demo2.dir/main.cpp.o.requires

CMakeFiles/demo2.dir/main.cpp.o.provides: CMakeFiles/demo2.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/demo2.dir/build.make CMakeFiles/demo2.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/demo2.dir/main.cpp.o.provides

CMakeFiles/demo2.dir/main.cpp.o.provides.build: CMakeFiles/demo2.dir/main.cpp.o


# Object files for target demo2
demo2_OBJECTS = \
"CMakeFiles/demo2.dir/main.cpp.o"

# External object files for target demo2
demo2_EXTERNAL_OBJECTS =

demo2: CMakeFiles/demo2.dir/main.cpp.o
demo2: CMakeFiles/demo2.dir/build.make
demo2: /usr/lib/x86_64-linux-gnu/libcxsparse.so
demo2: /usr/lib/x86_64-linux-gnu/libcholmod.so
demo2: CMakeFiles/demo2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/Project/CLionProject/G2O/demo2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable demo2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/demo2.dir/build: demo2

.PHONY : CMakeFiles/demo2.dir/build

CMakeFiles/demo2.dir/requires: CMakeFiles/demo2.dir/main.cpp.o.requires

.PHONY : CMakeFiles/demo2.dir/requires

CMakeFiles/demo2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/demo2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/demo2.dir/clean

CMakeFiles/demo2.dir/depend:
	cd /home/ubuntu/Project/CLionProject/G2O/demo2/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Project/CLionProject/G2O/demo2 /home/ubuntu/Project/CLionProject/G2O/demo2 /home/ubuntu/Project/CLionProject/G2O/demo2/cmake-build-debug /home/ubuntu/Project/CLionProject/G2O/demo2/cmake-build-debug /home/ubuntu/Project/CLionProject/G2O/demo2/cmake-build-debug/CMakeFiles/demo2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/demo2.dir/depend

