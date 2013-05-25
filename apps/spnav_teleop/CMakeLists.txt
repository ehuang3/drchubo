###############################################
# apps/publish_joint_commands tutorial

set(ROSBUILD_DONT_REDEFINE_PROJECT True)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(spnav_teleop_src spnav_teleop.cpp)
set(spnav_teleop_hdr )

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_add_executable(spnav_teleop ${spnav_teleop_src} ${spnav_teleop_hdr})
TARGET_LINK_LIBRARIES(spnav_teleop vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
set_target_properties(spnav_teleop PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")