###############################################
# apps/publish_joint_commands tutorial

set(ROSBUILD_DONT_REDEFINE_PROJECT True)
rosbuild_init()

file(GLOB cxx_srcs "*.cpp")
file(GLOB cxx_hdrs "*.h")

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_add_executable(ros_ik ${cxx_srcs} ${cxx_hdrs})
TARGET_LINK_LIBRARIES(ros_ik vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES})
set_target_properties(ros_ik PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")