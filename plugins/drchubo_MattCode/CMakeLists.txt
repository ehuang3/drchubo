# **************************************
# CMakeLists.txt for Matt's code
# (so hacky!)
# **************************************

include(mzsettings.cmake)

include_directories( . )

add_subdirectory(mzcommon)
add_subdirectory(src)
add_subdirectory(zmp)

# Find Dart
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
link_directories(${DART_LIBRARY_DIRS})

# Find expat needed for Matt's parsing
find_library( EXPAT_LIBRARY expat )

#set( CMAKE_SHARED_LINKER_FLAGS "-Wl -fPIC" )

add_library ( drchubo_MattCode SHARED drchubo_MattCode.cpp )
  
target_link_libraries (drchubo_MattCode ${GAZEBO_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${Boost_LIBRARIES} hubozmp hubomz mzcommon expat) 
  
set_target_properties( drchubo_MattCode PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )

