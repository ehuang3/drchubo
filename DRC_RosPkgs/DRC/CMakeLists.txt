# ***********************************************************************
# DRC 
# ROS package to simulate DRC-Hubo using animation of joints + poses
# ***********************************************************************
cmake_minimum_required(VERSION 2.4.6)

# Add the ROS .cmake
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Project
project(DRC)

# Build with ROS magic
rosbuild_init()

# Find Gazebo dependency
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

# Set Gazebo include/lib paths
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin/)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib/)
# Is this needed?
rosbuild_add_boost_directories()

# Important: Set drchubo_PATH!! Default is in the same directory as DRC
set( drchubo_ROOT_PATH ${CMAKE_SOURCE_DIR}/../drchubo )
message( WARNING "\n SETTING DRCHUBO PACKAGE PATH TO: ${drchubo_ROOT_PATH}\n CHANGE IT IF THIS PATH IS NOT CORRECT" )

# Generate path header file
set( DRC_PATH_HEADER ${CMAKE_SOURCE_DIR}/include/DRC_paths.h )
message( status "Generate the path header file ${DRC_PATH_HEADER}" )
configure_file( ${DRC_PATH_HEADER}.cmake ${DRC_PATH_HEADER} )

#Include
include_directories( ${CMAKE_SOURCE_DIR}/include )

# Generate library DRCPlugin
add_subdirectory( plugins/DRCPlugin )

# Generate ROS nodes

# Walk publisher executable
add_subdirectory( src/walkPublisher )

