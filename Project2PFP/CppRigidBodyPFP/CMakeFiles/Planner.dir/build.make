# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP

# Include any dependencies generated for this target.
include CMakeFiles/Planner.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Planner.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Planner.dir/flags.make

CMakeFiles/Planner.dir/src/Graphics.o: CMakeFiles/Planner.dir/flags.make
CMakeFiles/Planner.dir/src/Graphics.o: src/Graphics.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Planner.dir/src/Graphics.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Planner.dir/src/Graphics.o -c /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/src/Graphics.cpp

CMakeFiles/Planner.dir/src/Graphics.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Planner.dir/src/Graphics.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/src/Graphics.cpp > CMakeFiles/Planner.dir/src/Graphics.i

CMakeFiles/Planner.dir/src/Graphics.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Planner.dir/src/Graphics.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/src/Graphics.cpp -o CMakeFiles/Planner.dir/src/Graphics.s

CMakeFiles/Planner.dir/src/Graphics.o.requires:
.PHONY : CMakeFiles/Planner.dir/src/Graphics.o.requires

CMakeFiles/Planner.dir/src/Graphics.o.provides: CMakeFiles/Planner.dir/src/Graphics.o.requires
	$(MAKE) -f CMakeFiles/Planner.dir/build.make CMakeFiles/Planner.dir/src/Graphics.o.provides.build
.PHONY : CMakeFiles/Planner.dir/src/Graphics.o.provides

CMakeFiles/Planner.dir/src/Graphics.o.provides.build: CMakeFiles/Planner.dir/src/Graphics.o

CMakeFiles/Planner.dir/src/RigidBodySimulator.o: CMakeFiles/Planner.dir/flags.make
CMakeFiles/Planner.dir/src/RigidBodySimulator.o: src/RigidBodySimulator.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Planner.dir/src/RigidBodySimulator.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Planner.dir/src/RigidBodySimulator.o -c /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/src/RigidBodySimulator.cpp

CMakeFiles/Planner.dir/src/RigidBodySimulator.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Planner.dir/src/RigidBodySimulator.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/src/RigidBodySimulator.cpp > CMakeFiles/Planner.dir/src/RigidBodySimulator.i

CMakeFiles/Planner.dir/src/RigidBodySimulator.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Planner.dir/src/RigidBodySimulator.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/src/RigidBodySimulator.cpp -o CMakeFiles/Planner.dir/src/RigidBodySimulator.s

CMakeFiles/Planner.dir/src/RigidBodySimulator.o.requires:
.PHONY : CMakeFiles/Planner.dir/src/RigidBodySimulator.o.requires

CMakeFiles/Planner.dir/src/RigidBodySimulator.o.provides: CMakeFiles/Planner.dir/src/RigidBodySimulator.o.requires
	$(MAKE) -f CMakeFiles/Planner.dir/build.make CMakeFiles/Planner.dir/src/RigidBodySimulator.o.provides.build
.PHONY : CMakeFiles/Planner.dir/src/RigidBodySimulator.o.provides

CMakeFiles/Planner.dir/src/RigidBodySimulator.o.provides.build: CMakeFiles/Planner.dir/src/RigidBodySimulator.o

CMakeFiles/Planner.dir/src/RigidBodyPlanner.o: CMakeFiles/Planner.dir/flags.make
CMakeFiles/Planner.dir/src/RigidBodyPlanner.o: src/RigidBodyPlanner.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Planner.dir/src/RigidBodyPlanner.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Planner.dir/src/RigidBodyPlanner.o -c /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/src/RigidBodyPlanner.cpp

CMakeFiles/Planner.dir/src/RigidBodyPlanner.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Planner.dir/src/RigidBodyPlanner.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/src/RigidBodyPlanner.cpp > CMakeFiles/Planner.dir/src/RigidBodyPlanner.i

CMakeFiles/Planner.dir/src/RigidBodyPlanner.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Planner.dir/src/RigidBodyPlanner.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/src/RigidBodyPlanner.cpp -o CMakeFiles/Planner.dir/src/RigidBodyPlanner.s

CMakeFiles/Planner.dir/src/RigidBodyPlanner.o.requires:
.PHONY : CMakeFiles/Planner.dir/src/RigidBodyPlanner.o.requires

CMakeFiles/Planner.dir/src/RigidBodyPlanner.o.provides: CMakeFiles/Planner.dir/src/RigidBodyPlanner.o.requires
	$(MAKE) -f CMakeFiles/Planner.dir/build.make CMakeFiles/Planner.dir/src/RigidBodyPlanner.o.provides.build
.PHONY : CMakeFiles/Planner.dir/src/RigidBodyPlanner.o.provides

CMakeFiles/Planner.dir/src/RigidBodyPlanner.o.provides.build: CMakeFiles/Planner.dir/src/RigidBodyPlanner.o

# Object files for target Planner
Planner_OBJECTS = \
"CMakeFiles/Planner.dir/src/Graphics.o" \
"CMakeFiles/Planner.dir/src/RigidBodySimulator.o" \
"CMakeFiles/Planner.dir/src/RigidBodyPlanner.o"

# External object files for target Planner
Planner_EXTERNAL_OBJECTS =

bin/Planner: CMakeFiles/Planner.dir/src/Graphics.o
bin/Planner: CMakeFiles/Planner.dir/src/RigidBodySimulator.o
bin/Planner: CMakeFiles/Planner.dir/src/RigidBodyPlanner.o
bin/Planner: /usr/lib/i386-linux-gnu/libGL.so
bin/Planner: /usr/lib/i386-linux-gnu/libGLU.so
bin/Planner: /usr/lib/libglut.so
bin/Planner: CMakeFiles/Planner.dir/build.make
bin/Planner: CMakeFiles/Planner.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/Planner"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Planner.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Planner.dir/build: bin/Planner
.PHONY : CMakeFiles/Planner.dir/build

CMakeFiles/Planner.dir/requires: CMakeFiles/Planner.dir/src/Graphics.o.requires
CMakeFiles/Planner.dir/requires: CMakeFiles/Planner.dir/src/RigidBodySimulator.o.requires
CMakeFiles/Planner.dir/requires: CMakeFiles/Planner.dir/src/RigidBodyPlanner.o.requires
.PHONY : CMakeFiles/Planner.dir/requires

CMakeFiles/Planner.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Planner.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Planner.dir/clean

CMakeFiles/Planner.dir/depend:
	cd /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP /home/lordoku/school/CS692/Project2PFP/CppRigidBodyPFP/CMakeFiles/Planner.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Planner.dir/depend

