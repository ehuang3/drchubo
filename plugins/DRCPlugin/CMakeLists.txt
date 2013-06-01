# *****************************************************
# Add DRCPlugin library
# WorldPlugin similar to VRC Plugin, but for DRC-Hubo
# *****************************************************
set(ROSBUILD_DONT_REDEFINE_PROJECT True)
rosbuild_init()

rosbuild_add_library(DRCPlugin DRCPlugin.cpp)
target_link_libraries(DRCPlugin)

