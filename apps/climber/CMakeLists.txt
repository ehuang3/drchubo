set(ROSBUILD_DONT_REDEFINE_PROJECT True)
rosbuild_init()

rosbuild_add_executable( climber Climber.cpp demo.cpp )
set_target_properties( climber PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

