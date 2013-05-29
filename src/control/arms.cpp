#include "arms.h"
#include "utils/robot_configs.h"
#include <amino.h>


REGISTER_CONTROLLER(control::ARM_AIK_T, ARM_AIK)
REGISTER_CONTROLLER(control::ARM_AJIK_T, ARM_AJIK)
REGISTER_CONTROLLER(control::ARM_JIT_T, ARM_JIT)
REGISTER_CONTROLLER(control::ARM_JIK_T, ARM_JIK)

namespace control {

    bool ARM_AIK_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        if(!data)
            return false;

        // 1. Setup
        kinematics::Skeleton* robot = data->robot;
        robot::robot_kinematics_t* robot_kin = data->kin;
        Eigen::Isometry3d Twhand = data->manip_xform[data->manip_index];
        int mi = data->manip_index;

        // 2. Run IK
        bool ok = robot_kin->arm_ik(Twhand, mi == robot::MANIP_L_HAND, target);
        return ok;
    }

    bool ARM_AJIK_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        if(!data)
            return false;

        // 1. Setup
        robot::robot_kinematics_t* robot_kin = data->kin;
        Eigen::Isometry3d Twhand = data->manip_xform[data->manip_index];
        int mi = data->manip_index;

        // 2. Run IK
        bool ok = robot_kin->arm_jac_ik(Twhand, mi == robot::MANIP_L_HAND, target);
        return ok;
    }

    bool ARM_JIT_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        if(!data)
            return false;

        // 1. Setup
        kinematics::Skeleton* robot = data->robot;
        kinematics::BodyNode *end_effector = 
            robot->getNode(data->manip_index == robot::MANIP_L_HAND ? ROBOT_LEFT_HAND : ROBOT_RIGHT_HAND);
        std::vector<int> desired_dofs;
        target.get_manip_indexes(desired_dofs, data->manip_index);

        // 2. Jacobians setup
        robot::robot_jacobian_t* robot_jac = data->jac;
        Eigen::MatrixXd J;  // the forward jacobian
        Eigen::VectorXd dofs; // the configuration of the limb in jointspace
        Eigen::VectorXd qdot; // the necessary jointspace velocity of the limb
        Eigen::VectorXd& movement = data->joy_movement; // the cartesian velocty of the limb

        // 3. Compute the jacobian
        robot_jac->manip_jacobian(J, desired_dofs, end_effector, target);

        // 4. Use the jacobian to get the jointspace velocity
        qdot = Eigen::VectorXd::Zero(J.cols());
        aa_la_dls(J.rows(), J.cols(), 0.1, J.data(), movement.data(), qdot.data());
        
        // 5. Use the jointspace velocity to update the target position
        dofs.resize(desired_dofs.size());
        target.get_dofs(dofs, desired_dofs);
        target.set_dofs(dofs + qdot, desired_dofs);
        
        // 6. Clamp joint angles! Or else we are dead.
        bool ok = target.clamp_indexes(desired_dofs);
        return ok;
    }

    bool ARM_JIK_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        return false;
    }

    bool ARM_JPIT_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        return false;
    }

    bool ARM_JPIK_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        return false;
    }

}
