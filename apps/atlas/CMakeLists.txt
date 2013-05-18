###############################################
# apps/atlas

project(atlas)
file(GLOB atlas_srcs "*.cpp")
file(GLOB atlas_hdrs "*.h")
add_executable(atlas ${atlas_srcs} ${atlas_hdrs})
target_link_libraries(atlas dart_atlas ${DART_LIBRARIES} ${DARTExt_LIBRARIES})

set_target_properties(atlas PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")