set(ROSBUILD_DONT_REDEFINE_PROJECT True)
rosbuild_init()

rosbuild_add_executable( grabImage grabImage.cpp )
target_link_libraries( grabImage )
set_target_properties( grabImage PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )

rosbuild_add_executable( grabPointCloud2 grabPointCloud2.cpp )
target_link_libraries( grabPointCloud2 )
set_target_properties( grabPointCloud2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )