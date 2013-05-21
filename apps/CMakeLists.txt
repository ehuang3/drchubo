# A list of applications
set_property(DIRECTORY PROPERTY FOLDER Apps)
set_property(DIRECTORY PROPERTY EXCLUDE_FROM_DEFAULT_BUILD ON)

# List of all the subdirectories to include
foreach(APPDIR
    ros_ik
    robot_viz
    drcsim_joint_commands_tutorial
    quadruped
    climber
    )
    add_subdirectory(${APPDIR})
endforeach(APPDIR)
