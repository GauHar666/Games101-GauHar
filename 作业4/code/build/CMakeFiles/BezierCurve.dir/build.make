# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/melodic/桌面/Games101/作业4/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/melodic/桌面/Games101/作业4/code/build

# Include any dependencies generated for this target.
include CMakeFiles/BezierCurve.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/BezierCurve.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BezierCurve.dir/flags.make

CMakeFiles/BezierCurve.dir/main.cpp.o: CMakeFiles/BezierCurve.dir/flags.make
CMakeFiles/BezierCurve.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/melodic/桌面/Games101/作业4/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BezierCurve.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BezierCurve.dir/main.cpp.o -c /home/melodic/桌面/Games101/作业4/code/main.cpp

CMakeFiles/BezierCurve.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BezierCurve.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/melodic/桌面/Games101/作业4/code/main.cpp > CMakeFiles/BezierCurve.dir/main.cpp.i

CMakeFiles/BezierCurve.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BezierCurve.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/melodic/桌面/Games101/作业4/code/main.cpp -o CMakeFiles/BezierCurve.dir/main.cpp.s

# Object files for target BezierCurve
BezierCurve_OBJECTS = \
"CMakeFiles/BezierCurve.dir/main.cpp.o"

# External object files for target BezierCurve
BezierCurve_EXTERNAL_OBJECTS =

BezierCurve: CMakeFiles/BezierCurve.dir/main.cpp.o
BezierCurve: CMakeFiles/BezierCurve.dir/build.make
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
BezierCurve: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
BezierCurve: CMakeFiles/BezierCurve.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/melodic/桌面/Games101/作业4/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable BezierCurve"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BezierCurve.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BezierCurve.dir/build: BezierCurve

.PHONY : CMakeFiles/BezierCurve.dir/build

CMakeFiles/BezierCurve.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BezierCurve.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BezierCurve.dir/clean

CMakeFiles/BezierCurve.dir/depend:
	cd /home/melodic/桌面/Games101/作业4/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/melodic/桌面/Games101/作业4/code /home/melodic/桌面/Games101/作业4/code /home/melodic/桌面/Games101/作业4/code/build /home/melodic/桌面/Games101/作业4/code/build /home/melodic/桌面/Games101/作业4/code/build/CMakeFiles/BezierCurve.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BezierCurve.dir/depend

