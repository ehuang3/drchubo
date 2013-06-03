###############################################
# apps/pelvis_tilt_demo

project(pelvis_tilt_demo)
file(GLOB app_srcs "*.cpp")
file(GLOB app_hdrs "*.h")
add_executable(pelvis_tilt_demo ${app_srcs} ${app_hdrs})
target_link_libraries(pelvis_tilt_demo vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES})

set_target_properties(pelvis_tilt_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")