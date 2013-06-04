cmake_minimum_required( VERSION 2.8 FATAL_ERROR )

include( FindPkgConfig )
if( PKG_CONFIG_FOUND )
  pkg_check_modules( GAZEBO gazebo )
endif()

include_directories( ${GAZEBO_INCLUDE_DIRS} )
message( status "Gazebo include dirs: ${GAZEBO_INCLUDE_DIRS}" )
link_directories( ${GAZEBO_LIBRARY_DIRS} /usr/lib )
add_executable( convertURDF convertURDF.cpp )
target_link_libraries( convertURDF ${GAZEBO_libraries} gazebo_sdf_interface tinyxml )
