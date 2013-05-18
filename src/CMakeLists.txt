project(dart-atlas)

# Enable multi-threaded compilation.
# We do this here and not in the root folder since the example apps
# do not have enough source files to benefit from this.
# if(MSVC)
# 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
# endif()

# Setup directory structure
set(SUBDIRS atlas utils)
foreach(SUBDIR ${SUBDIRS})
	file(GLOB_RECURSE dir_files ${SUBDIR}/*.h ${SUBDIR}/*.hpp ${SUBDIR}/*.cpp)
	source_group(${SUBDIR} FILES ${dir_files})
endforeach()

# Search all header and source files
file(GLOB_RECURSE dart_atlas_srcs *.cpp)
file(GLOB_RECURSE dart_atlas_hdrs *.h *.hpp)

# Library
add_library(dart_atlas ${dart_atlas_srcs} ${dart_atlas_hdrs})
target_link_libraries(dart_atlas ${DART_LIBRARIES} ${DARTExt_LIBRARIES})
set_target_properties(dart_atlas PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")

# if(MSVC)
#    set_target_properties(${target} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
# endif()

# install(DIRECTORY ./ DESTINATION include/dart 
#         FILES_MATCHING PATTERN "*.h*" 
#         PATTERN "*CMake*" EXCLUDE)
#         
# install(TARGETS dart DESTINATION lib)