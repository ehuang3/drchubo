# **************************
# Qlistener test
# **************************

message( status "SOURCE DIR FOR LISTENER: ${CMAKE_SOURCE_DIR}")
include_directories( ${CMAKE_SOURCE_DIR}/rosQtInterface )

# Add qlistener test
add_subdirectory( qlistener )
add_subdirectory( qtalker )
add_subdirectory( qimuListener )
add_subdirectory( qpcl2Listener )