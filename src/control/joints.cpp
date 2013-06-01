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

        robot->setPose(target.dart_pose());
        data->manip_target = link->getWorldTransform();
        
        return ok;
    }

}
