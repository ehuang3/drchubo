# ***********************************************************************
# DRC 
# ROS package to simulate DRC-Hubo using animation of joints + poses
# ***********************************************************************
cmake_minimum_required (VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project (drchubo)

# ROS build setup - No!
# set(ROSBUILD_DONT_REDEFINE_PROJECT True)
# rosbuild_init()

# find Gazebo dependency
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

# Set Gazebo include/lib paths
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# drchubo helpful messsages
message(STATUS "Creating Makefile for drchubo")
message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

# Build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo Profile."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Color GCC option
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUCPP)
  option(COLOR_GCC "Use GCCFilter to color compiler output messages" OFF)
   set(COLOR_GCC_OPTIONS "-c -r -w -p -n")
   if(COLOR_GCC)
      set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_SOURCE_DIR}/tools/gccfilter ${COLOR_GCC_OPTIONS}")
   endif()
endif()

# Compile flags
if(WIN32)
  set (CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
  message (STATUS "Setup Visual Studio Specific Flags")
  set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
  set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc /openmp")
  set (CMAKE_DEBUG_POSTFIX "d")
  set (CMAKE_EXE_LINKER_FLAGS "/LTCG")
  set (CMAKE_MODULE_LINKER_FLAGS "/LTCG")
else ()
  set(CMAKE_INSTALL_PREFIX /usr )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -std=gnu++0x -g")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
endif()

set(CMAKE_DEBUG_POSTFIX "d")

# Generate the path header file
set(PATH_HEADER_DIR ${CMAKE_SOURCE_DIR}/src/utils/data_paths.h)
message(STATUS "Generate the path header file to ${PATH_HEADER_DIR}")
configure_file(${PATH_HEADER_DIR}.in ${PATH_HEADER_DIR} @ONLY)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin/)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib/)

# Link with DART 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
link_directories(${DART_LIBRARY_DIRS})

# Build source
# 0. 
message(STATUS "Building drchubo source")
include_directories(BEFORE src)
link_directories(${CMAKE_SOURCE_DIR}/lib)
add_subdirectory(src)

# 1. Create main DRCPlugin
add_subdirectory( plugins )

# 2. Generate apps (ROS nodes) which mostly depend on DRCPlugin
add_subdirectory( apps )

#message(STATUS "Building drchubo source")
#include_directories(BEFORE src)
#link_directories(${CMAKE_SOURCE_DIR}/lib)
#add_subdirectory(src)

#enable_testing()
#add_subdirectory(test)
#add_subdirectory(apps)
