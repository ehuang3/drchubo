set(ROSBUILD_DONT_REDEFINE_PROJECT True)
rosbuild_init()

rosbuild_add_executable( climber Climber.cpp demo.cpp )
set_target_properties( climber PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

rosbuild_add_executable( trajDemo trajTesterDemo.cpp trajTester.cpp )
target_link_libraries( trajDemo vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES} )
set_target_properties( trajDemo PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )
