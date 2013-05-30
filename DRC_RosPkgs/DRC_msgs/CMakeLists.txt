cmake_minimum_required(VERSION 2.4.6)

# Add the ROS .cmake
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Note: recovering PROJECT_NAME here to drcsim leads to build failure
project(DRC_msgs)

# Build with ROS magic
rosbuild_init()

# Generate our messages
rosbuild_genmsg()


