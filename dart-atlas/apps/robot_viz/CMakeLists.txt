###############################################
# apps/robot_viz

project(robot_viz)
file(GLOB app_srcs "*.cpp")
file(GLOB app_hdrs "*.h")
add_executable(robot_viz ${app_srcs} ${app_hdrs})
target_link_libraries(robot_viz dart_atlas ${DART_LIBRARIES} ${DARTExt_LIBRARIES})

set_target_properties(robot_viz PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")