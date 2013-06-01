###############################################
# apps/teleop-demo
set(ROSBUILD_DONT_REDEFINE_PROJECT True)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROSBUILD_TYPE DEBUG)
rosbuild_init()

set(hubo_teleop_src teleop_demo.cpp spnav.cpp Fastrak.cpp hubo_fastrak.cpp)
set(hubo_teleop_hdr )

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_add_executable(hubo_teleop ${hubo_teleop_src} ${hubo_teleop_hdr})

TARGET_LINK_LIBRARIES(hubo_teleop vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
set_target_properties(hubo_teleop PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
target_link_libraries(hubo_teleop m ach c rt pthread protobuf z somatic protobuf-c)