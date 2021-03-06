#include "arms.h"
#include "utils/robot_configs.h"
#include <amino.h>
#include <assert.h>


namespace control {

    bool ARM_AIK_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        if(!data)
            return false;

        // 1. Setup
        kinematics::Skeleton* robot = data->robot;
        robot::robot_kinematics_t* robot_kin = data->kin;
        int ms = data->manip_side;
        int mi = ms ? robot::MANIP_L_HAND : robot::MANIP_R_HAND;
        Eigen::Isometry3d Twhand;
        data->get_IK_target(Twhand, mi, data->ik_target);

        // 1 Save joint angles to check for later
        Eigen::VectorXd q_prev;
        target.get_manip(q_prev, mi);

        // 2. Add delta transform
        Twhand.linear() = Twhand.linear() * data->joy_rotation;
        Twhand.translation() += data->joy_position;

        // 3. Convert to global frame
        data->convert_to_global(Twhand, target, mi, data->ik_target);

        // 4.. Run IK
        bool ok = robot_kin->arm_ik(Twhand, ms, target);

        // 5.. Visualize target
        data->manip_target = Twhand;

        // Check joint angles for large discontinuity
        Eigen::VectorXd q_new;
        target.get_manip(q_new, mi);

        std::cout << "AIK delta = " << (q_new-q_prev).norm() << std::endl;

        if((q_new - q_prev).norm() > 0.1)
            ok = false;

        return ok;
    }

    bool ARM_SENSOR_AIK_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        if(!data)
            return false;

        // 1. Setup
        kinematics::Skeleton* robot = data->robot;
        robot::robot_kinematics_t* robot_kin = data->kin;
        int ms = data->manip_side;
        int mi = ms ? robot::MANIP_L_HAND : robot::MANIP_R_HAND;
        Eigen::Isometry3d Twhand;
        data->get_IK_target(Twhand, mi, data->ik_target);

        // 0. save
        Eigen::VectorXd q(7);
        target.get_manip(q, mi);

        // 2. Add delta transform
        Twhand = Twhand * data->sensor_tf[ms];

        // 3. Convert to global frame
        data->convert_to_global(Twhand, target, mi, data->ik_target);

        // 4. Run IK
        bool ok = robot_kin->arm_ik(Twhand, ms, target);

        // 3. Visualize target
        data->manip_target = Twhand;

        // %. fix bad stuff
        if(!ok)
            target.set_manip(q, mi);

        return true; //< Never fill IK targets when running fastrak
    }

    bool ARM_MASTER_SLAVE_AIK_T::change_mode(int key, robot::robot_state_t& target)
    {
        do_init = 1;
    }

    bool ARM_MASTER_SLAVE_AIK_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        assert(data);

        // Require that the slave transform is relative between the fake HUBO wrist pitches
        if (do_init) {
            std::cout << "[ARM_MASTER_SLAVE_AIK] Attaching slave to master\n";

            Eigen::Isometry3d Tf_rwp, Tf_lwp;
            data->get_IK_target(Tf_rwp, robot::MANIP_R_HAND, control::GLOBAL_MANIP);
            data->get_IK_target(Tf_lwp, robot::MANIP_L_HAND, control::GLOBAL_MANIP);

            Tf_slave = Tf_rwp.inverse() * Tf_lwp;

            do_init = 0;
        }

        // Master hand
        Eigen::Isometry3d Tf_left, Tf_right;
        data->get_IK_target(Tf_right, robot::MANIP_R_HAND, control::GLOBAL_MANIP);

        // Delta motion
        Tf_right.linear() = Tf_right.linear() * data->joy_rotation;
        Tf_right.translation() += data->joy_position;

        // Slave hand
        Tf_left = Tf_right * Tf_slave;

        // Do IK
        robot::robot_kinematics_t* robot_kin = data->kin;
        bool ok;

        ok = robot_kin->arm_ik(Tf_right, false, target);
        ok &= robot_kin->arm_ik(Tf_left, true, target);

        data->manip_target = Tf_right;

        return ok;
    }

    bool ARM_DUAL_AIK_T::run(robot::robot_state_t& target, control::control_data_t* data)
    {
        // save
        int ms = data->manip_side;

        bool ok = true;

        data->manip_side = 0;
        ok &= sensor_aik.run(target, data);
        data->manip_side = 1;
        ok &= sensor_aik.run(target, data);

        data->manip_side = ms;

        return true;
    }

    bool ARM_AJIK_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        if(!data)
            return false;
        if(!data->joystick_ok)
            return false;

        // 1. Setup
        robot::robot_kinematics_t* robot_kin = data->kin;
        int ms = data->manip_side;
        int mi = ms ? robot::MANIP_L_HAND : robot::MANIP_R_HAND;
        Eigen::Isometry3d Twhand = data->manip_xform[mi];

        // 2. Add delta transform
        Twhand.linear() = Twhand.linear() * data->joy_rotation;
        Twhand.translation() += data->joy_position;

        // 2. Run IK
        bool ok = robot_kin->arm_jac_ik(Twhand, ms, target);

        // 3. Save new manip
        if (ok)
            data->manip_xform[mi] = Twhand;

        // 3. Visualize the target
        data->manip_target = Twhand;

        return ok;
    }

    bool ARM_JIT_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        if(!data)
            return false;

        if(!data->joystick_ok)
            return false;

        // 1. Setup
        int ms = data->manip_side;
        int mi = ms ? robot::MANIP_L_HAND : robot::MANIP_R_HAND;
        kinematics::Skeleton* robot = data->robot;
        kinematics::BodyNode *end_effector = 
            robot->getNode(ms ? ROBOT_LEFT_HAND : ROBOT_RIGHT_HAND);
        std::vector<int> desired_dofs;
        target.get_manip_indexes(desired_dofs, mi);

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
        bool ok = target.clamp_indexes(desired_dofs, false);

        // 7. Set target manip correctly so other controllers can use it
        robot::robot_kinematics_t* robot_kin = data->kin;
        Eigen::Isometry3d Twhand_hubo; //< Get "Hubo" hand location

        robot_kin->arm_fk(Twhand_hubo, ms, target);
        data->manip_xform[mi] = Twhand_hubo;
        data->manip_target = Twhand_hubo;

        return ok;
    }

    void jit_sensor(int ms, robot::robot_state_t& target, control_data_t* data)
    {
        int mi = ms ? robot::MANIP_L_HAND : robot::MANIP_R_HAND;
        kinematics::Skeleton* robot = data->robot;
        kinematics::BodyNode *end_effector = 
            robot->getNode(ms ? ROBOT_LEFT_HAND : ROBOT_RIGHT_HAND);
        std::vector<int> desired_dofs;

        robot->setPose(target.dart_pose());

        target.get_manip_indexes(desired_dofs, mi);

        // 2. Jacobians setup
        robot::robot_jacobian_t* robot_jac = data->jac;
        Eigen::MatrixXd J;  // the forward jacobian
        Eigen::VectorXd dofs; // the configuration of the limb in jointspace
        Eigen::VectorXd qdot; // the necessary jointspace velocity of the limb
        Eigen::Vector6d movement; // the cartesian velocty of the limb

        // Fill out movement
        Eigen::Isometry3d target_xform = data->sensor_tf[ms];
        
        Eigen::Isometry3d current_xform;
        current_xform = end_effector->getWorldTransform();
        
        robot_jac->xform_error(movement, target_xform, current_xform);

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
        bool ok = target.clamp_indexes(desired_dofs, false);

        // 7. Set target manip correctly so other controllers can use it
        robot::robot_kinematics_t* robot_kin = data->kin;
        Eigen::Isometry3d Twhand; //< Get hand location

        robot_kin->arm_fk(Twhand, ms, target);
        data->manip_xform[mi] = Twhand;
        data->manip_target = target_xform;
    }

    bool ARM_BOTH_JIT_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        assert(data);
        if(!data->sensor_ok)
            return false;

        //std::cout << "Left sensor position = " << data->sensor_position[1].transpose() << std::endl;

        robot::robot_jacobian_t* robot_jac = data->jac;

        for(int i=0; i < 2; i++) {
            jit_sensor(i, target, data);
        }

        return true;
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
