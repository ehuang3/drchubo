##########################
# zmp_steps
set(ROSBUILD_DONT_REDEFINE_PROJECT True)
rosbuild_init()

# Build library adaptedZMP
add_subdirectory( adaptedZMP )
include_directories( . )
rosbuild_add_executable( zmp_steps zmp_steps.cpp )
target_link_libraries( zmp_steps ${DART_LIBRARIES} ${DARTExt_LIBRARIES} vrc_golem adaptedZMP )
set_target_properties( zmp_steps PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )

rosbuild_add_executable( comikTest comikTest.cpp )
target_link_libraries( comikTest ${DART_LIBRARIES} ${DARTExt_LIBRARIES} vrc_golem adaptedZMP )
set_target_properties( comikTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )


rosbuild_add_executable( comIK comIK.cpp )
target_link_libraries( comIK ${DART_LIBRARIES} ${DARTExt_LIBRARIES} vrc_golem )
set_target_properties( comIK PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )
