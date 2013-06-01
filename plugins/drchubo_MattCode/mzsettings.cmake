set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

if(APPLE)
  include_directories(/opt/local/include) # MacPorts
  find_library(OPENGL_LIBRARY OpenGL)
else()
  find_library(OPENGL_LIBRARY GL)
  find_library(GLU_LIBRARY GLU)
  set(OPENGL_LIBRARY ${OPENGL_LIBRARY} ${GLU_LIBRARY})
endif()

find_library(EXPAT_LIBRARY expat)
find_library(GLUT_LIBRARY glut)

include(FindPkgConfig)

find_package(Qt4 4.4 COMPONENTS QtCore QtGui QtOpenGL Qt3Support QtNetwork)
pkg_search_module(EIGEN3 REQUIRED eigen3>=3)

include_directories(${EIGEN3_INCLUDE_DIRS})

if (QT_FOUND)
  add_definitions(${QT_DEFINITIONS})
  if (QT4_FOUND)
    add_definitions(-DMZ_HAVE_QT4)
    include(${QT_USE_FILE})
  endif (QT4_FOUND)
endif (QT_FOUND)

set(CMAKE_C_FLAGS "-Wall -g")
set(CMAKE_CXX_FLAGS "-Wall -g")

if(APPLE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wsign-compare")
  set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wsign-compare")
endif()

set(CMAKE_C_FLAGS_DEBUG "-O")
set(CMAKE_CXX_FLAGS_DEBUG "-O")

set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

macro(add_gui_app name)
  if(APPLE)
    add_executable(${name} MACOSX_BUNDLE ${ARGN})
  else()
    add_executable(${name} ${ARGN})
  endif()
endmacro(add_gui_app)

include_directories(${PROJECT_SOURCE_DIR})

find_path(HUBO_ACH_INCLUDE "hubo.h" PATHS ${CMAKE_INCLUDE_PATH})

if (${HUBO_ACH_INCLUDE} STREQUAL HUBO_ACH_INCLUDE-NOTFOUND)
  message("could not find hubo-ach includes")
  set(HAVE_HUBO_ACH 0)
else (${HUBO_ACH_INCLUDE} STREQUAL HUBO_ACH_INCLUDE-NOTFOUND)
  message("found hubo-ach includes at ${HUBO_ACH_INCLUDE}")
 # add_definitions(-DHAVE_HUBO_ACH)
  set(HAVE_HUBO_ACH 1)
endif (${HUBO_ACH_INCLUDE} STREQUAL HUBO_ACH_INCLUDE-NOTFOUND)
