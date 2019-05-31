# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/iprl/apps/Camera/tracker

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/iprl/apps/Camera/tracker/build

# Include any dependencies generated for this target.
include src/CMakeFiles/tracker.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/tracker.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/tracker.dir/flags.make

src/CMakeFiles/tracker.dir/tracker.cpp.o: src/CMakeFiles/tracker.dir/flags.make
src/CMakeFiles/tracker.dir/tracker.cpp.o: ../src/tracker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/iprl/apps/Camera/tracker/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/tracker.dir/tracker.cpp.o"
	cd /home/iprl/apps/Camera/tracker/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracker.dir/tracker.cpp.o -c /home/iprl/apps/Camera/tracker/src/tracker.cpp

src/CMakeFiles/tracker.dir/tracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracker.dir/tracker.cpp.i"
	cd /home/iprl/apps/Camera/tracker/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/iprl/apps/Camera/tracker/src/tracker.cpp > CMakeFiles/tracker.dir/tracker.cpp.i

src/CMakeFiles/tracker.dir/tracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracker.dir/tracker.cpp.s"
	cd /home/iprl/apps/Camera/tracker/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/iprl/apps/Camera/tracker/src/tracker.cpp -o CMakeFiles/tracker.dir/tracker.cpp.s

# Object files for target tracker
tracker_OBJECTS = \
"CMakeFiles/tracker.dir/tracker.cpp.o"

# External object files for target tracker
tracker_EXTERNAL_OBJECTS =

../bin/tracker: src/CMakeFiles/tracker.dir/tracker.cpp.o
../bin/tracker: src/CMakeFiles/tracker.dir/build.make
../bin/tracker: /usr/local/lib/libopencv_dnn.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_gapi.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_highgui.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_ml.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_objdetect.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_photo.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_stitching.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_video.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_videoio.so.4.1.0
../bin/tracker: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/tracker: /usr/local/lib/librealsense2.so.2.22.0
../bin/tracker: /home/iprl/core/sai2-common/build/libsai2-common.a
../bin/tracker: /home/iprl/core/chai3d/build/libchai3d.a
../bin/tracker: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/tracker: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/tracker: /home/iprl/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/tracker: /home/iprl/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/tracker: /home/iprl/core/sai2-simulation-master/build/libsai2-simulation.a
../bin/tracker: /home/iprl/core/sai2-model/build/libsai2-model.a
../bin/tracker: /home/iprl/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/tracker: /home/iprl/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/tracker: /home/iprl/core/sai2-model/rbdl/build/librbdl.so
../bin/tracker: /home/iprl/core/sai2-graphics/build/libsai2-graphics.a
../bin/tracker: /home/iprl/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/tracker: /home/iprl/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/tracker: /home/iprl/core/chai3d/build/libchai3d.a
../bin/tracker: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/tracker: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/tracker: /home/iprl/core/sai2-primitives/build/libsai2-primitives.a
../bin/tracker: /home/iprl/core/sai2-primitives/ReflexxesTypeII/Linux/x64/release/lib/shared/libReflexxesTypeII.so
../bin/tracker: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/tracker: /usr/lib/x86_64-linux-gnu/libglfw.so
../bin/tracker: /usr/local/lib/libopencv_imgcodecs.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_calib3d.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_features2d.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_flann.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_imgproc.so.4.1.0
../bin/tracker: /usr/local/lib/libopencv_core.so.4.1.0
../bin/tracker: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/tracker: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/tracker: /home/iprl/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/tracker: /home/iprl/core/sai2-model/rbdl/build/librbdl.so
../bin/tracker: /home/iprl/core/sai2-primitives/ReflexxesTypeII/Linux/x64/release/lib/shared/libReflexxesTypeII.so
../bin/tracker: /usr/lib/x86_64-linux-gnu/libglfw.so
../bin/tracker: src/CMakeFiles/tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/iprl/apps/Camera/tracker/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/tracker"
	cd /home/iprl/apps/Camera/tracker/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/tracker.dir/build: ../bin/tracker

.PHONY : src/CMakeFiles/tracker.dir/build

src/CMakeFiles/tracker.dir/clean:
	cd /home/iprl/apps/Camera/tracker/build/src && $(CMAKE_COMMAND) -P CMakeFiles/tracker.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/tracker.dir/clean

src/CMakeFiles/tracker.dir/depend:
	cd /home/iprl/apps/Camera/tracker/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/iprl/apps/Camera/tracker /home/iprl/apps/Camera/tracker/src /home/iprl/apps/Camera/tracker/build /home/iprl/apps/Camera/tracker/build/src /home/iprl/apps/Camera/tracker/build/src/CMakeFiles/tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/tracker.dir/depend

