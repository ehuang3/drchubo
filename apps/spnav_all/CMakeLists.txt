###############################################
# apps/spnav-all

set(ROSBUILD_DONT_REDEFINE_PROJECT True)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(spnav_all_src spnav_all.cpp MyWindow.cpp)
set(spnav_all_hdr )

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_add_executable(spnav_all ${spnav_all_src} ${spnav_all_hdr})
TARGET_LINK_LIBRARIES(spnav_all vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
set_target_properties(spnav_all PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")