
set( CMAKE_SHARED_LINKER_FLAGS "-fPIC" )

add_library(hubozmp zmpwalkgenerator.cpp footprint.cpp gait-timer.cpp swing.cpp)
set_target_properties( hubozmp PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )


set(zmpdemo_libs hubozmp hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)

# Add an example
add_gui_app(zmpdemo zmpdemo.cpp)
target_link_libraries(zmpdemo ${zmpdemo_libs})
set_target_properties( zmpdemo PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )
