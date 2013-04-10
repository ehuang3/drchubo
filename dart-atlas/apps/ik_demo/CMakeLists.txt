###############################################
# apps/ik_demo

project(ik_demo)
file(GLOB ik_demo_srcs "*.cpp")
file(GLOB ik_demo_hdrs "*.h")
add_executable(ik_demo ${ik_demo_srcs} ${ik_demo_hdrs})
target_link_libraries(ik_demo dart_atlas ${DART_LIBRARIES} ${DARTExt_LIBRARIES})

set_target_properties(ik_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")