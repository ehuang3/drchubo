###################################
# CMake for drchuboBasics_plugin
###################################

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} . drchubo_Basics ) # Last one for control 
link_directories(${GAZEBO_LIBRARY_DIRS})

set( control_srcs drchubo_Basics/controls/pid_controller.cpp drchubo_Basics/controls/controlBundle.cpp )

#########################
# drchubo_Basics
#########################
add_library( drchubo_Basics SHARED drchubo_Basics/drchubo_Basics.cpp ${control_srcs} ) 
target_link_libraries( drchubo_Basics ${GAZEBO_LIBRARIES} )

set_target_properties( drchubo_Basics PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )

#########################
# drchubo_Animation
#########################
add_library( drchubo_Animation SHARED drchubo_Animation/drchubo_Animation.cpp ) 
target_link_libraries( drchubo_Animation ${GAZEBO_LIBRARIES} )

set_target_properties( drchubo_Animation PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )

#########################
# drchubo_AnimationPush
#########################
add_library( drchubo_AnimationPush SHARED drchubo_AnimationPush/drchubo_AnimationPush.cpp ) 
target_link_libraries( drchubo_AnimationPush ${GAZEBO_LIBRARIES} )

set_target_properties( drchubo_AnimationPush PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )

#########################
# drchubo_Walk1
#########################
add_library( drchubo_Walk1 SHARED drchubo_Walk1/drchubo_Walk1.cpp ${control_srcs} ) 
target_link_libraries( drchubo_Walk1 ${GAZEBO_LIBRARIES} )

set_target_properties( drchubo_Walk1 PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )

#########################
# drchubo_MattCode
#########################
add_subdirectory( drchubo_MattCode )
