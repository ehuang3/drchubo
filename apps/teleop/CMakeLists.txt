###############################################
# apps/teleop-demo

set(ROSBUILD_DONT_REDEFINE_PROJECT True)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROSBUILD_TYPE DEBUG)
rosbuild_init()



set(teleop_demo_src teleop_demo.cpp spnav.cpp)
set(teleop_demo_hdr )

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_add_executable(teleop_demo ${teleop_demo_src} ${teleop_demo_hdr})
TARGET_LINK_LIBRARIES(teleop_demo vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
set_target_properties(teleop_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

rosbuild_remove_compile_flags(teleop_demo "-O3")