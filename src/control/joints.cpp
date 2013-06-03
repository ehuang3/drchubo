#include "joints.h"
#include "utils/robot_configs.h"
#include <iostream>
#include <assert.h>


namespace control {

    bool JOINT_T::change_mode(int ch, robot::robot_state_t& target)
    {
        kinematics::Skeleton* robot = target.robot();
        joint_index = ch;
        joint_index = ch % target.dart_pose().size();
        std::cout << "Controlling dof " << robot->getDof(joint_index)->getName() << std::endl;

        return true;
    }

    bool JOINT_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        assert(data);

        // 1. Setup
        kinematics::Skeleton* robot = target.robot();
        kinematics::BodyNode* link = robot->getDof(joint_index)->getJoint()->getChildNode();
        
        double omega = data->joy_movement[5]; // z-axis
        target.dofs(joint_index) += omega;
        bool ok = target.clamp_dof(joint_index, false);

        assert(target.check_limits());

        robot->setPose(target.dart_pose());
        data->manip_target = link->getWorldTransform();

        return ok;
    }

    bool TORSO_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        int joint_index = target.get_index("TSY");
        
        double omega = data->joy_movement[5]; // z-axis
        target.dofs(joint_index) += omega;
        bool ok = target.clamp_dof(joint_index, false);

        return ok;
    }

    bool GRASP_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        std::vector<int> left_hand, right_hand;
        target.get_manip_indexes(left_hand, robot::LIMB_L_HAND);
        target.get_manip_indexes(right_hand, robot::LIMB_R_HAND);
        int joint_index = left_hand[0];

        kinematics::Skeleton* robot = target.robot();
        kinematics::BodyNode* link = robot->getDof(joint_index)->getJoint()->getChildNode();
        
        double omega = data->joy_movement[5];
        bool ok = true;
        for(int i=0; i < left_hand.size(); i++) {
            int d = left_hand[i];
            target.dofs(d) += omega;
            ok &= target.clamp_dof(d, false);
        }
        for(int i=0; i < left_hand.size(); i++) {
            int d = right_hand[i];
            target.dofs(d) += omega;
            ok &= target.clamp_dof(d, false);
        }

        assert(target.check_limits());

        return ok;
    }

    bool FLOATING_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        // 1. Setup
        Eigen::Isometry3d body;
        target.get_body(body);

        // 1. Variables
        Eigen::Quaterniond save_rotation(data->joy_rotation);
        Eigen::Vector3d save_position = data->joy_position;
        
        Eigen::Quaterniond rotation(data->joy_rotation);
        Eigen::Vector3d& position = data->joy_position;

        // 2. Clamp to xyy
        position[2] = 0;
        rotation.y() = 0;
        rotation.x() = 0;

        body.linear() = body.linear() * rotation.matrix();
        body.translation() += position;

        // 3.
        target.set_body(body);

        return true;
    }    

}
