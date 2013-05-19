###############################################
# apps/publish_joint_commands tutorial

set(ROSBUILD_DONT_REDEFINE_PROJECT True)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(quadruped_enter_src quadruped_enter.cpp)
set(quadruped_enter_hdr )

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_add_executable(quadruped_enter ${quadruped_enter_src} ${quadruped_enter_hdr})
TARGET_LINK_LIBRARIES(quadruped_enter vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
set_target_properties(quadruped_enter PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")