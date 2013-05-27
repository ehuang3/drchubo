###############################################
# apps/publish_joint_commands tutorial

set(ROSBUILD_DONT_REDEFINE_PROJECT True)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(spnav_com_src spnav_com.cpp)
set(spnav_com_hdr )

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_add_executable(spnav_com ${spnav_com_src} ${spnav_com_hdr})
TARGET_LINK_LIBRARIES(spnav_com vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
set_target_properties(spnav_com PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")