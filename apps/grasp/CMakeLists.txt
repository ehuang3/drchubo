# ************************************************************
# zmpnode
# ROS nodet that generates trajectories based on Matt's code
# ************************************************************
set(ROSBUILD_DONT_REDEFINE_PROJECT True)
rosbuild_init()

rosbuild_add_executable ( grasp grasp.cpp )  
set_target_properties( grasp PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin" )


