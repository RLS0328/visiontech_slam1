cmake_minimum_required(VERSION 2.8.1)

project(ethzasl_msf_noros)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules/)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/3rdparty/;${PROJECT_SOURCE_DIR}/3rdparty/lib/cmake;${CMAKE_PREFIX_PATH}")

enable_testing()

find_package(Eigen3 REQUIRED)
#find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
find_package(Glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${GTEST_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
if (CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif ()


add_subdirectory(src)
