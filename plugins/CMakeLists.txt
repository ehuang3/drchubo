###################################
# CMake for drchuboBasics_plugin
###################################

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} . ) 
link_directories(${GAZEBO_LIBRARY_DIRS})

set( control_srcs controls/pid_controller.cpp controls/controlBundle.cpp )


add_library( drchuboBasics_plugin SHARED drchubo_Basics.cpp ${control_srcs} ) # pose_animation.cpp 
target_link_libraries( drchuboBasics_plugin ${GAZEBO_LIBRARIES} )

set_target_properties( drchuboBasics_plugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )
