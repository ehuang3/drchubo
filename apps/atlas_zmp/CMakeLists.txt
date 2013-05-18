###############################################
# apps/atlas_zmp

project(atlas_zmp)
file(GLOB atlas_zmp_srcs "*.cpp")
file(GLOB atlas_zmp_hdrs "*.h")
add_executable(atlas_zmp ${atlas_zmp_srcs} ${atlas_zmp_hdrs})
target_link_libraries(atlas_zmp dart_atlas ${DART_LIBRARIES} ${DARTExt_LIBRARIES})

set_target_properties(atlas_zmp PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")