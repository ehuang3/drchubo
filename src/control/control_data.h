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
    //### want to use. Add fields as needed, go crazy.
    //############################################################

    struct control_data_t {
        //############################################################
        // Teleop fields
        //############################################################
        // Mode of control .. maybe unused
        robot::TeleopMode teleop_mode;

        // Joystick commands
        Eigen::VectorXd joy_raw; //< raw joystick
        Eigen::VectorXd joy_filtered; //< filtered joystick
        Eigen::VectorXd joy_movement; //< filtered and time derivated joystick 

        //. target xform of the end effector
        Eigen::Isometry3d manip_target;
        Eigen::Isometry3d manip_prev;

        //. array of target xforms (stance ik)
        Eigen::Isometry3d manip_xform[robot::NUM_MANIPULATORS];
        robot::IK_Mode manip_mode[robot::NUM_MANIPULATORS];
        int manip_index;
        
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
