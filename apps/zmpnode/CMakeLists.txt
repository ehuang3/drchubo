# ************************************************************
# zmpnode
# ROS nodet that generates trajectories based on Matt's code
# ************************************************************
set(ROSBUILD_DONT_REDEFINE_PROJECT True)
rosbuild_init()

include( matt_stuff/mzsettings.cmake)

# I should fix this so we don't add matt_stuff as include necessarily
include_directories( . matt_stuff )

add_subdirectory( matt_stuff/mzcommon )
add_subdirectory( matt_stuff/src )
add_subdirectory( matt_stuff/zmp )

# Find Dart
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/dart")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
link_directories(${DART_LIBRARY_DIRS})

# Find expat needed for Matt's parsing
find_library( EXPAT_LIBRARY expat )

rosbuild_add_executable ( zmpnode main.cpp zmpnode.cpp )  
target_link_libraries (zmpnode hubozmp hubomz mzcommon expat ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${Boost_LIBRARIES}) 
set_target_properties( zmpnode PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )

#rosbuild_add_executable ( zmpnode_arbitrary main_TestArbitraryStart.cpp zmpnode.cpp )  
#target_link_libraries (zmpnode_arbitrary hubozmp hubomz mzcommon expat ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${Boost_LIBRARIES}) 
#set_target_properties( zmpnode_arbitrary PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )

rosbuild_add_executable ( zmpWalk_smoothTransition zmpWalk_smoothTransition.cpp zmpnode.cpp )  
target_link_libraries (zmpWalk_smoothTransition hubozmp hubomz mzcommon expat ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${Boost_LIBRARIES}) 
set_target_properties( zmpWalk_smoothTransition PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )


#rosbuild_add_executable ( continuousWalk continuousWalk.cpp )  
#target_link_libraries (continuousWalk hubozmp hubomz mzcommon expat ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${Boost_LIBRARIES}) 
#set_target_properties( continuousWalk PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )

