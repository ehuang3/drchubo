###############################################
# apps/spnav-all

set(ROSBUILD_DONT_REDEFINE_PROJECT True)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(fastrak_teleop_src Teleop.cpp spnav_all.cpp)
set(fastrak_teleop_hdr)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_add_executable(fastrak_teleop ${fastrak_teleop_src} ${fastrak_teleop_hdr})

target_link_libraries(fastrak_teleop m ach c rt pthread protobuf z somatic protobuf-c)

TARGET_LINK_LIBRARIES(fastrak_teleop vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
set_target_properties(fastrak_teleop PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")


set_target_properties(fastrak_teleop PROPERTIES CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")