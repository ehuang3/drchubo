# ***************************************************************
# adaptedZMP 
# Code translated from Matt Zucker hubozmp
# using atlas-lib from Eric and 
# getting rid of mzcommon code and using Eigen types instead
# @author: A. Huaman 
# @date 2013/05/10
# ***************************************************************
add_library( adaptedZMP zmpWalkGenerator.cpp
  swing.cpp
  gait-timer.cpp
  footprint.cpp
  AtlasKinematics_Extra.cpp )

target_link_libraries( adaptedZMP vrc_golem )
set_target_properties( adaptedZMP PROPERTIES 
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )
