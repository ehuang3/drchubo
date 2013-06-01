#pragma once
#include <Eigen/Dense>
#include "robot/robot.h"
#include "robot/robot_state.h"
#include "robot/robot_kinematics.h"
#include "robot/robot_jacobian.h"
#include <kinematics/Skeleton.h>

namespace control {

    //############################################################
    //### This structure holds all the fields that controllers
    //### want to use. Add fields as needed
    //############################################################

    struct control_data_t {
        //############################################################
        // Teleop fields
        //############################################################
        // Mode of control .. maybe unused
        robot::TeleopMode teleop_mode;

        // Joystick commands
        Eigen::VectorXd buttons; //< buttons that are depressed
        Eigen::VectorXd triggers; //< buttons that have been toggled
        Eigen::VectorXd joy_raw; //< raw joystick
        Eigen::VectorXd joy_filtered; //< filtered joystick
        Eigen::VectorXd joy_movement; //< filtered and time derivated joystick

        Eigen::Matrix3d joy_rotation;
        Eigen::Vector3d joy_position;
        bool joystick_ok;

        // Sensor data
        Eigen::Matrix3d sensor_rotation;
        Eigen::Vector3d sensor_position;
        bool sensor_ok;

        //############################################################
        //### Control fields
        //############################################################
        // target xform of the end effector
        Eigen::Isometry3d manip_target;
        Eigen::Isometry3d manip_prev;

        // target com
        Eigen::Vector3d com;

        // array of target xforms (stance ik)
        Eigen::Isometry3d manip_xform[robot::NUM_MANIPULATORS];
        robot::IK_Mode manip_mode[robot::NUM_MANIPULATORS];
        int manip_index;
        int manip_side; //< left = 1 

        // last command
        Eigen::VectorXd last_command;
        
        //############################################################
        // General fields
        //############################################################
        // The robot skeleton to use
        kinematics::Skeleton* robot;

        // The current state of the robot
        robot::robot_state_t *current;

        // Various kinematics solvers
        robot::robot_kinematics_t *kin;
        robot::robot_jacobian_t *jac;
    };

}
