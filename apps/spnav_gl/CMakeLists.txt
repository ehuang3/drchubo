###############################################
# apps/spnav_gl

set(ROSBUILD_DONT_REDEFINE_PROJECT True)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(spnav_gl_src spnav_gl.cpp MyWindow.cpp)
set(spnav_gl_hdr )

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_add_executable(spnav_gl ${spnav_gl_src} ${spnav_gl_hdr})
TARGET_LINK_LIBRARIES(spnav_gl vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
set_target_properties(spnav_gl PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")