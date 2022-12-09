cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)
SET(CMAKE_BUILD_TYPE Debug)
##-DCMAKE_BUILD_TYPE=Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
#msse3
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -fPIC -O3 -Wall -march=native -DEIGEN_DONT_PARALLELIZE -g -ggdb ")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -march=native")
SET(CMAKE_C_FLAGS_DEBUG "$ENV{CXXFLAGS} -O3 -Wall -g -ggdb")
SET(CMAKE_C_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(MSF_INCLUDE /home/lee/MSF/src/msf_updates/pose_position_msf)
set(MSF_LIB /home/lee/MSF/lib)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(mynteyed REQUIRED)
if(mynteyed_WITH_OPENCV)
   add_definitions(-DWITH_OPENCV)
endif()
#find_package(Boost					# Minimum or EXACT version e.g. 1.36.0
#  REQUIRED             					# Fail with error if Boost is not found
#  COMPONENTS serialization 				# Boost libraries by their canonical name
#  )
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2)
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/g2o)

set(OSG_INCLUDE /home/lee/3dParty/OpenSceneGraph-OpenSceneGraph-3.4.0/include)
set(OSG_LIB /home/lee/3dParty/OpenSceneGraph-OpenSceneGraph-3.4.0/build/lib)
set(MYNTEYE_LIB /home/lee/MYNT-EYE-D-SDK/_output/lib)
set(VISLAM_AR_PATH ${PROJECT_SOURCE_DIR}/Examples/AR)

include_directories(${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${OSG_INCLUDE}
${VISLAM_AR_PATH}/include
        ${MSF_INCLUDE}
)
# Build tools
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary)
add_executable(bin_vocabulary
Vocabulary/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${PROJECT_NAME})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
        #src/SystemSetting.cc
        #src/InitKeyFrame.cc
        src/MapSaveLoad.cc
        src/Pickhandle.cc

Examples/AR/src/Obj.cc
Examples/AR/src/Loader.cc
Examples/AR/src/CameraManipulator.cc
Examples/AR/src/AR.cc

#Examples/AR/src/ViewerAR.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
#${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${OSG_LIB}/libosg.so
${OSG_LIB}/libosgDB.so
${OSG_LIB}/libosgGA.so
${OSG_LIB}/libosgUtil.so
${OSG_LIB}/libosgViewer.so
${MSF_LIB}/libMSF.so
#${MSF_LIB}/libmsf_core.so
#${MSF_LIB}/libmsf_timing.so
#${MSF_LIB}/libpose_distorter.so
#${OSG_LIB}/osgPlugins-3.6.0/osgdb_ive.so
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/AR)

add_executable(mono_ar
Examples/AR/src/mono_ar.cc)
target_link_libraries(mono_ar ${PROJECT_NAME} -lpthread -lm)
target_link_libraries(mono_ar mynteye_depth ${OpenCV_LIBS})
add_executable(mono
        Examples/AR/src/mono.cc)

target_link_libraries(mono ${PROJECT_NAME} -lpthread -lm)

# Build examples

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

#add_executable(rgbd_tum
#Examples/RGB-D/rgbd_tum.cc)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

#add_executable(stereo_kitti
#Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

#add_executable(stereo_euroc
#Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME})


#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

#add_executable(mono_tum
#Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_kitti
#Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})

#add_executable(mono_euroc
#Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME})
