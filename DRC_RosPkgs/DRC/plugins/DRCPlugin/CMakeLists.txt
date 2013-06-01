# *****************************************************
# Add DRCPlugin library
# WorldPlugin similar to VRC Plugin, but for DRC-Hubo
# *****************************************************
rosbuild_add_library(DRCPlugin DRCPlugin.cpp)
target_link_libraries(DRCPlugin)

