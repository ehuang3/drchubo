#include "control_data.h"

namespace control {

    void control_data_t::fill_IK_targets(robot::robot_state_t& robot, control::IK_Target ik)
    {
        kinematics::Skeleton *robot_skel = robot.robot();
        robot_skel->setPose(robot.dart_pose());

        std::vector<int> manip_index = { robot::MANIP_L_HAND, robot::MANIP_R_HAND, robot::MANIP_L_FOOT, robot::MANIP_R_FOOT };
        std::vector<int> manip_link = { ROBOT_LEFT_HAND, ROBOT_RIGHT_HAND, ROBOT_LEFT_FOOT, ROBOT_RIGHT_FOOT };
        Eigen::Isometry3d B, Tf;
        if (ik & control::UARM_MANIP) {
            // Save dofs and zero out arms (specifically L/RSR)
            Eigen::VectorXd save = robot.dart_pose();
            Eigen::VectorXd q(6);
            q.setZero();
            robot.set_manip(q, robot::MANIP_L_HAND);
            robot.set_manip(q, robot::MANIP_R_HAND);

            robot_skel->setPose(robot.dart_pose());

            // Fill manip array
            std::vector<const char*> uarm_link = { "Body_LSR", "Body_RSR", "Body_LSR", "Body_RSR" };
            for(int i=0; i < robot::NUM_MANIPULATORS; i++) {
                // Calculate B relative to upper arm
                Tf = robot_skel->getNode(uarm_link[i])->getWorldTransform();
                // B is the target manip
                B = robot_skel->getNode(manip_link[i])->getWorldTransform();
                // B' is relative to upper arm
                Tf_arm_manip[manip_index[i]] = Tf.inverse() * B;
            }

            // Restore original pose
            robot.set_dart_pose(save);
            robot_skel->setPose(robot.dart_pose());
        }
        if (ik & control::BODY_MANIP) {
            Tf = robot_skel->getNode(ROBOT_BODY)->getWorldTransform();
            for(int i=0; i < robot::NUM_MANIPULATORS; i++) {
                // B is the target manip
                B = robot_skel->getNode(manip_link[i])->getWorldTransform();
                // B' is relative to body
                Tf_body_manip[manip_index[i]] = Tf.inverse() * B;
            }
        }
        if (ik & control::GLOBAL_MANIP) {
            for(int i=0; i < robot::NUM_MANIPULATORS; i++) {
                Tf_global_manip[manip_index[i]] = robot_skel->getNode(manip_link[i])->getWorldTransform();
            }
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
        }
        if(ik & control::GLOBAL_MANIP) {
            B = Tf_global_manip[mi];
        }
    }

}
