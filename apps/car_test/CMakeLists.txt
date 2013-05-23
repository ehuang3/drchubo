set(ROSBUILD_DONT_REDEFINE_PROJECT True)
rosbuild_init()

rosbuild_add_executable( car_test car_test.cpp )
set_target_properties( car_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

