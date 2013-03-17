# Simple CMakeLists file for a shared library
# by Andrew Price
#
# Compiles cpp files in a source directory to a .so library file
#

# Project Description
cmake_minimum_required(VERSION 2.6)
project(GolemsVRCLib)

# Include the OpenCV library
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_libraries(${OpenCV_LIBS})

# Include the project files
include_directories("include")

# Collect the source, script and test files
file(GLOB lib_source "source/*.cpp")
LIST(SORT lib_source)

# Build library
add_library(${PROJECT_NAME} SHARED ${lib_source})

