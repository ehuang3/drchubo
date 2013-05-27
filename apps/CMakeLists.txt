# A list of applications
set_property(DIRECTORY PROPERTY FOLDER Apps)
set_property(DIRECTORY PROPERTY EXCLUDE_FROM_DEFAULT_BUILD ON)

# List of all the subdirectories to include
foreach(APPDIR
    ros_ik
    robot_viz
    #drcsim_joint_commands_tutorial
    quadruped
    climber
    zmp_steps
    car_test
    perception
    spnav_teleop
    pelvis_tilt_demo
    spnav_com
    )
    add_subdirectory(${APPDIR})
endforeach(APPDIR)
