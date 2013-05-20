###############################################
# apps/zmp_driver

project(zmp_driver)
file(GLOB cxx_srcs "*.cpp")
file(GLOB cxx_hdrs "*.h")
add_executable(zmp_driver ${cxx_srcs} ${cxx_hdrs})
target_link_libraries(zmp_driver dart_atlas ${DART_LIBRARIES} ${DARTExt_LIBRARIES})

set_target_properties(zmp_driver PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")