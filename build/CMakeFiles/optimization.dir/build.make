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
CMAKE_SOURCE_DIR = /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/build

# Include any dependencies generated for this target.
include CMakeFiles/optimization.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/optimization.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/optimization.dir/flags.make

CMakeFiles/optimization.dir/src/optimization/cam_localization.o: CMakeFiles/optimization.dir/flags.make
CMakeFiles/optimization.dir/src/optimization/cam_localization.o: ../src/optimization/cam_localization.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/optimization.dir/src/optimization/cam_localization.o"
	/usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optimization.dir/src/optimization/cam_localization.o -c /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/src/optimization/cam_localization.cpp

CMakeFiles/optimization.dir/src/optimization/cam_localization.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optimization.dir/src/optimization/cam_localization.i"
	/usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/src/optimization/cam_localization.cpp > CMakeFiles/optimization.dir/src/optimization/cam_localization.i

CMakeFiles/optimization.dir/src/optimization/cam_localization.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optimization.dir/src/optimization/cam_localization.s"
	/usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/src/optimization/cam_localization.cpp -o CMakeFiles/optimization.dir/src/optimization/cam_localization.s

CMakeFiles/optimization.dir/src/optimization/cam_localization.o.requires:

.PHONY : CMakeFiles/optimization.dir/src/optimization/cam_localization.o.requires

CMakeFiles/optimization.dir/src/optimization/cam_localization.o.provides: CMakeFiles/optimization.dir/src/optimization/cam_localization.o.requires
	$(MAKE) -f CMakeFiles/optimization.dir/build.make CMakeFiles/optimization.dir/src/optimization/cam_localization.o.provides.build
.PHONY : CMakeFiles/optimization.dir/src/optimization/cam_localization.o.provides

CMakeFiles/optimization.dir/src/optimization/cam_localization.o.provides.build: CMakeFiles/optimization.dir/src/optimization/cam_localization.o


CMakeFiles/optimization.dir/src/optimization/sm_mapping.o: CMakeFiles/optimization.dir/flags.make
CMakeFiles/optimization.dir/src/optimization/sm_mapping.o: ../src/optimization/sm_mapping.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/optimization.dir/src/optimization/sm_mapping.o"
	/usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optimization.dir/src/optimization/sm_mapping.o -c /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/src/optimization/sm_mapping.cpp

CMakeFiles/optimization.dir/src/optimization/sm_mapping.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optimization.dir/src/optimization/sm_mapping.i"
	/usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/src/optimization/sm_mapping.cpp > CMakeFiles/optimization.dir/src/optimization/sm_mapping.i

CMakeFiles/optimization.dir/src/optimization/sm_mapping.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optimization.dir/src/optimization/sm_mapping.s"
	/usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/src/optimization/sm_mapping.cpp -o CMakeFiles/optimization.dir/src/optimization/sm_mapping.s

CMakeFiles/optimization.dir/src/optimization/sm_mapping.o.requires:

.PHONY : CMakeFiles/optimization.dir/src/optimization/sm_mapping.o.requires

CMakeFiles/optimization.dir/src/optimization/sm_mapping.o.provides: CMakeFiles/optimization.dir/src/optimization/sm_mapping.o.requires
	$(MAKE) -f CMakeFiles/optimization.dir/build.make CMakeFiles/optimization.dir/src/optimization/sm_mapping.o.provides.build
.PHONY : CMakeFiles/optimization.dir/src/optimization/sm_mapping.o.provides

CMakeFiles/optimization.dir/src/optimization/sm_mapping.o.provides.build: CMakeFiles/optimization.dir/src/optimization/sm_mapping.o


# Object files for target optimization
optimization_OBJECTS = \
"CMakeFiles/optimization.dir/src/optimization/cam_localization.o" \
"CMakeFiles/optimization.dir/src/optimization/sm_mapping.o"

# External object files for target optimization
optimization_EXTERNAL_OBJECTS =

liboptimization.so: CMakeFiles/optimization.dir/src/optimization/cam_localization.o
liboptimization.so: CMakeFiles/optimization.dir/src/optimization/sm_mapping.o
liboptimization.so: CMakeFiles/optimization.dir/build.make
liboptimization.so: /usr/local/lib/libg2o_core.so
liboptimization.so: /usr/local/lib/libg2o_stuff.so
liboptimization.so: /usr/local/lib/libg2o_cli.so
liboptimization.so: /usr/local/lib/libg2o_solver_cholmod.so
liboptimization.so: /usr/local/lib/libg2o_solver_csparse.so
liboptimization.so: /usr/local/lib/libg2o_csparse_extension.so
liboptimization.so: /usr/local/lib/libg2o_solver_dense.so
liboptimization.so: /usr/local/lib/libg2o_solver_pcg.so
liboptimization.so: /usr/local/lib/libg2o_solver_slam2d_linear.so
liboptimization.so: /usr/local/lib/libg2o_solver_structure_only.so
liboptimization.so: /usr/local/lib/libg2o_solver_eigen.so
liboptimization.so: /usr/local/lib/libg2o_types_data.so
liboptimization.so: /usr/local/lib/libg2o_types_icp.so
liboptimization.so: /usr/local/lib/libg2o_types_sba.so
liboptimization.so: /usr/local/lib/libg2o_types_sclam2d.so
liboptimization.so: /usr/local/lib/libg2o_types_sim3.so
liboptimization.so: /usr/local/lib/libg2o_types_slam2d.so
liboptimization.so: /usr/local/lib/libg2o_types_slam3d.so
liboptimization.so: /usr/lib/x86_64-linux-gnu/libcholmod.so
liboptimization.so: /usr/lib/x86_64-linux-gnu/libamd.so
liboptimization.so: /usr/lib/x86_64-linux-gnu/libcolamd.so
liboptimization.so: /usr/lib/x86_64-linux-gnu/libcamd.so
liboptimization.so: /usr/lib/x86_64-linux-gnu/libccolamd.so
liboptimization.so: /usr/local/lib/libmetis.so
liboptimization.so: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
liboptimization.so: /usr/lib/x86_64-linux-gnu/libcxsparse.so
liboptimization.so: /usr/local/lib/libopencv_superres.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudaobjdetect.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudastereo.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_stitching.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudacodec.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_videostab.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudabgsegm.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_dpm.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_stereo.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_xobjdetect.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_rgbd.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_ccalib.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_xphoto.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_aruco.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_reg.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_img_hash.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_bgsegm.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_saliency.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_face.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_hdf.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_line_descriptor.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_optflow.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_structured_light.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_bioinspired.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_hfs.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_fuzzy.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_surface_matching.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_freetype.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_sfm.so.3.4.1
liboptimization.so: libutils.so
liboptimization.so: /usr/local/lib/libopencv_cudafeatures2d.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudaoptflow.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudalegacy.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudawarping.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_objdetect.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_photo.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudaimgproc.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudafilters.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_tracking.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_plot.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_datasets.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_text.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_ximgproc.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_viz.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_dnn.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_xfeatures2d.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_shape.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_calib3d.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudaarithm.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_features2d.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_ml.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_highgui.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_videoio.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_imgcodecs.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_flann.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_video.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_imgproc.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_core.so.3.4.1
liboptimization.so: /usr/local/lib/libopencv_cudev.so.3.4.1
liboptimization.so: CMakeFiles/optimization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library liboptimization.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optimization.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/optimization.dir/build: liboptimization.so

.PHONY : CMakeFiles/optimization.dir/build

CMakeFiles/optimization.dir/requires: CMakeFiles/optimization.dir/src/optimization/cam_localization.o.requires
CMakeFiles/optimization.dir/requires: CMakeFiles/optimization.dir/src/optimization/sm_mapping.o.requires

.PHONY : CMakeFiles/optimization.dir/requires

CMakeFiles/optimization.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/optimization.dir/cmake_clean.cmake
.PHONY : CMakeFiles/optimization.dir/clean

CMakeFiles/optimization.dir/depend:
	cd /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/build /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/build /home/luis/Documents/DRIVE/LUIS/CODIGOS/Smart_Markers/build/CMakeFiles/optimization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/optimization.dir/depend

