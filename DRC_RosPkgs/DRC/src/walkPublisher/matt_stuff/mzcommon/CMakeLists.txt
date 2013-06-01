# CMakeLists.txt for hubomz/mzcommon

set(mzcommon_srcs
  mersenne.c
  gauss.c
  GlCamera.cpp
  MzGlutApp.cpp
  TinyDom.cpp
  TriMesh3.cpp
  glstuff.cpp)

add_library( mzcommon ${mzcommon_srcs} )
set_target_properties( mzcommon PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )

