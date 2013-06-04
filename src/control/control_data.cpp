#include "control_data.h"
#include "utils/robot_configs.h"
#include <kinematics/BodyNode.h>
#include <iostream>

namespace control {

    void control_data_t::fill_IK_target(robot::robot_state_t& robot, int mi, control::IK_Target ik)
    {
        kinematics::Skeleton *robot_skel = robot.robot();
        robot_skel->setPose(robot.dart_pose());

        // FIXME: This is slow?
        std::vector<int> manip_index = { robot::MANIP_L_HAND, robot::MANIP_R_HAND, robot::MANIP_L_FOOT, robot::MANIP_R_FOOT };
        std::vector<const char*> manip_link(4, "");
        manip_link[0] = ROBOT_LEFT_HAND;
        manip_link[1] = ROBOT_RIGHT_HAND;
        manip_link[2] = ROBOT_LEFT_FOOT;
        manip_link[3] = ROBOT_RIGHT_FOOT;

        // Find target transforms of manipulators
        robot::robot_kinematics_t* robot_kin = this->kin;
        Eigen::Isometry3d Tf_manip[robot::NUM_MANIPULATORS];
        // Arm AIK requires HUBO fake location
        robot_kin->arm_fk(Tf_manip[robot::MANIP_L_HAND], true, robot, true);
        robot_kin->arm_fk(Tf_manip[robot::MANIP_R_HAND], false, robot, true);
        // Leg AIK requires HUBO real location
        Tf_manip[ manip_index[2] ] = robot_skel->getNode( manip_link[2] )->getWorldTransform();
        Tf_manip[ manip_index[3] ] = robot_skel->getNode( manip_link[3] )->getWorldTransform();

        Eigen::Isometry3d B, Tf;
        if (ik & control::UARM_MANIP) {
            // Save dofs and zero out arms (specifically L/RSR) to get the shoulder frame
            Eigen::VectorXd save = robot.dart_pose();
            Eigen::VectorXd q(6);
            q.setZero();
            robot.set_manip(q, robot::MANIP_L_HAND);
            robot.set_manip(q, robot::MANIP_R_HAND);
            robot_skel->setPose(robot.dart_pose());

            // Fill relative to shoulder manipulator array
            std::vector<const char*> uarm_link = { "Body_LSR", "Body_RSR", "Body_LSR", "Body_RSR" };
            Tf = robot_skel->getNode(uarm_link[mi])->getWorldTransform();
            Tf_uarm_manip[mi] = Tf.inverse() * Tf_manip[mi];

            // Restore original pose
            robot.set_dart_pose(save);
            robot_skel->setPose(robot.dart_pose());
        }
        if (ik & control::BODY_MANIP) {
            // Find manipulators relative to body
            Tf = robot_skel->getNode(ROBOT_BODY)->getWorldTransform();
            Tf_body_manip[mi] = Tf.inverse() * Tf_manip[mi];
        }
        if (ik & control::GLOBAL_MANIP) {
            Tf_global_manip[mi] = Tf_manip[mi];
        }
    }

    void control_data_t::get_IK_target(Eigen::Isometry3d& B, int mi, int ik)
    {
        if(!(ik ^ control::ALL_MANIP)) {
            std::cerr << "Called `get_IK_target` with ALL_MANIP" << std::endl;
            return;
        }
        if(ik & control::UARM_MANIP) {
            B = Tf_uarm_manip[mi];
            return;
        }
        if(ik & control::BODY_MANIP) {
            B = Tf_body_manip[mi];
            return;
        }
        if(ik & control::GLOBAL_MANIP) {
            B = Tf_global_manip[mi];
            return;
        }
    }

    void control_data_t::convert_to_global(Eigen::Isometry3d& B, robot::robot_state_t& robot, int mi, int ik) 
    {
        if(!(ik ^ control::ALL_MANIP)) {
            std::cerr << "Called `get_IK_target` with ALL_MANIP" << std::endl;
            return;
        }
        kinematics::Skeleton* robot_skel = robot.robot();
        if(ik & control::UARM_MANIP) {
            // Save dofs and zero out arms (specifically L/RSR) to get the shoulder frame
            Eigen::VectorXd save = robot.dart_pose();
            Eigen::VectorXd q(6);
            q.setZero();
            robot.set_manip(q, robot::MANIP_L_HAND);
            robot.set_manip(q, robot::MANIP_R_HAND);
            robot_skel->setPose(robot.dart_pose());

            // Fill relative to shoulder manipulator array
            std::vector<const char*> uarm_link = { "Body_LSR", "Body_RSR", "Body_LSR", "Body_RSR" };
            Eigen::Isometry3d Tf_uarm;
            Tf_uarm = robot_skel->getNode(uarm_link[mi])->getWorldTransform();
            B = Tf_uarm * B;

            // Restore original pose
            robot.set_dart_pose(save);
            robot_skel->setPose(robot.dart_pose());
            return;
        }
        if(ik & control::BODY_MANIP) {
            robot_skel->setPose(robot.dart_pose());
            Eigen::Isometry3d Tf_body;
            Tf_body = robot_skel->getNode(ROBOT_BODY)->getWorldTransform();            
            B = Tf_body * B;
            return;
        }
        if(ik & control::GLOBAL_MANIP) {
            return;
        }        
    }

}
