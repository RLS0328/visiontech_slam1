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
CMAKE_COMMAND = /home/lee/JetBrians/clion-2017.3.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/lee/JetBrians/clion-2017.3.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lee/桌面/visiontech_slam1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lee/桌面/visiontech_slam1/cmake-build-release

# Include any dependencies generated for this target.
include CMakeFiles/bin_vocabulary.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/bin_vocabulary.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bin_vocabulary.dir/flags.make

CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o: CMakeFiles/bin_vocabulary.dir/flags.make
CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o: ../Vocabulary/bin_vocabulary.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lee/桌面/visiontech_slam1/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o -c /home/lee/桌面/visiontech_slam1/Vocabulary/bin_vocabulary.cc

CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lee/桌面/visiontech_slam1/Vocabulary/bin_vocabulary.cc > CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.i

CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lee/桌面/visiontech_slam1/Vocabulary/bin_vocabulary.cc -o CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.s

CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o.requires:

.PHONY : CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o.requires

CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o.provides: CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o.requires
	$(MAKE) -f CMakeFiles/bin_vocabulary.dir/build.make CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o.provides.build
.PHONY : CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o.provides

CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o.provides.build: CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o


# Object files for target bin_vocabulary
bin_vocabulary_OBJECTS = \
"CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o"

# External object files for target bin_vocabulary
bin_vocabulary_EXTERNAL_OBJECTS =

../Vocabulary/bin_vocabulary: CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o
../Vocabulary/bin_vocabulary: CMakeFiles/bin_vocabulary.dir/build.make
../Vocabulary/bin_vocabulary: ../lib/libORB_SLAM2.so
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_dnn.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_ml.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_objdetect.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_shape.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_stitching.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_superres.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_videostab.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_calib3d.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_features2d.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_flann.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_highgui.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_photo.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_video.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_videoio.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_imgcodecs.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_imgproc.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_viz.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libopencv_core.so.3.3.0
../Vocabulary/bin_vocabulary: /usr/local/lib/libpangolin.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libGLU.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libGL.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libpython3.5m.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libavcodec.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libavformat.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libavutil.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libswscale.so
../Vocabulary/bin_vocabulary: /usr/lib/libOpenNI.so
../Vocabulary/bin_vocabulary: /usr/lib/libOpenNI2.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libpng.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libz.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libtiff.so
../Vocabulary/bin_vocabulary: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../Vocabulary/bin_vocabulary: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Vocabulary/bin_vocabulary: ../Thirdparty/g2o/lib/libg2o.so
../Vocabulary/bin_vocabulary: /home/lee/3dParty/OpenSceneGraph-OpenSceneGraph-3.4.0/build/lib/libosg.so
../Vocabulary/bin_vocabulary: /home/lee/3dParty/OpenSceneGraph-OpenSceneGraph-3.4.0/build/lib/libosgDB.so
../Vocabulary/bin_vocabulary: /home/lee/3dParty/OpenSceneGraph-OpenSceneGraph-3.4.0/build/lib/libosgGA.so
../Vocabulary/bin_vocabulary: /home/lee/3dParty/OpenSceneGraph-OpenSceneGraph-3.4.0/build/lib/libosgUtil.so
../Vocabulary/bin_vocabulary: /home/lee/3dParty/OpenSceneGraph-OpenSceneGraph-3.4.0/build/lib/libosgViewer.so
../Vocabulary/bin_vocabulary: /home/lee/MSF/lib/libMSF.so
../Vocabulary/bin_vocabulary: CMakeFiles/bin_vocabulary.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lee/桌面/visiontech_slam1/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Vocabulary/bin_vocabulary"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bin_vocabulary.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bin_vocabulary.dir/build: ../Vocabulary/bin_vocabulary

.PHONY : CMakeFiles/bin_vocabulary.dir/build

CMakeFiles/bin_vocabulary.dir/requires: CMakeFiles/bin_vocabulary.dir/Vocabulary/bin_vocabulary.cc.o.requires

.PHONY : CMakeFiles/bin_vocabulary.dir/requires

CMakeFiles/bin_vocabulary.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bin_vocabulary.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bin_vocabulary.dir/clean

CMakeFiles/bin_vocabulary.dir/depend:
	cd /home/lee/桌面/visiontech_slam1/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lee/桌面/visiontech_slam1 /home/lee/桌面/visiontech_slam1 /home/lee/桌面/visiontech_slam1/cmake-build-release /home/lee/桌面/visiontech_slam1/cmake-build-release /home/lee/桌面/visiontech_slam1/cmake-build-release/CMakeFiles/bin_vocabulary.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bin_vocabulary.dir/depend
