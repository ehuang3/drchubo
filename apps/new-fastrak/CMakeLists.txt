###############################################
# apps/new_fastrak

project(new_fastrak)
file(GLOB app_srcs "*.cpp")
file(GLOB app_hdrs "*.h")
add_executable(new_fastrak ${app_srcs} ${app_hdrs})
target_link_libraries(new_fastrak vrc_golem ${DART_LIBRARIES} ${DARTExt_LIBRARIES})

target_link_libraries(new_fastrak m ach c rt pthread protobuf z somatic protobuf-c)

set_target_properties(new_fastrak PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")