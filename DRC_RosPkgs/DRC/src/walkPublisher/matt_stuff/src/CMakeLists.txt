# CMakeLists.txt for hubomz/src


add_library( hubomz fakerave.cpp fakerave_parse.cpp HuboPlus.cpp HuboKin.cpp )
target_link_libraries( hubomz ${OPENGL_LIBRARY} ${GLUT_LIBRARY} )
set_target_properties( hubomz PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )
