#include "pelvis.h"
#include <utils/robot_configs.h>
#include <assert.h>

namespace control {

    bool BODY_XYZRPY_FIX_LEGS_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        assert(data);

        if(!data->joystick_ok)
            return false;

        // 1. Init
        kinematics::Skeleton* robot = data->robot;
        robot::robot_kinematics_t* robot_kin = data->kin;
        Eigen::Isometry3d (&end_effectors)[robot::NUM_MANIPULATORS] = data->manip_xform;
        robot::IK_Mode mode[robot::NUM_MANIPULATORS];
        mode[robot::MANIP_L_HAND] = robot::IK_MODE_FREE;
        mode[robot::MANIP_R_HAND] = robot::IK_MODE_FREE;
        mode[robot::MANIP_L_FOOT] = robot::IK_MODE_WORLD;
        mode[robot::MANIP_R_FOOT] = robot::IK_MODE_WORLD;
        Eigen::Isometry3d Tpelvis;
        target.get_body(Tpelvis);

        Eigen::VectorXd save = target.dart_pose();
        
        // 2. Compute target pelvis xform
        Tpelvis.linear() = Tpelvis.linear() * data->joy_rotation;
        Tpelvis.translation() += data->joy_position;

        target.set_body(Tpelvis);
        
        // 3. Run IK
        bool ok = robot_kin->stance_ik(end_effectors, mode, target);
        
        assert(target.check_limits());

        if (!ok)
            target.set_dart_pose(save);

        // 4. Save arm world xforms
        robot->setPose(target.dart_pose());
        end_effectors[robot::MANIP_L_HAND] = robot->getNode(ROBOT_LEFT_HAND)->getWorldTransform();
        end_effectors[robot::MANIP_R_HAND] = robot->getNode(ROBOT_RIGHT_HAND)->getWorldTransform();
        
        // 5. Viz target
        data->manip_target = Tpelvis;

        return ok;
    }

    bool BODY_XZP_FIX_LEGS_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        assert(data);

        if(!data->joystick_ok)
            return false;

        // 1. Variables
        Eigen::Quaterniond save_rotation(data->joy_rotation);
        Eigen::Vector3d save_position = data->joy_position;
        
        Eigen::Quaterniond rotation(data->joy_rotation);
        Eigen::Vector3d& position = data->joy_position;

        // 2. Clamp to xzp
        position[1] = 0;
        rotation.z() = 0;
        rotation.x() = 0;
        
        data->joy_rotation = rotation;

        // 3. Run
        bool ok = xyzrpy_fix_legs.run(target, data);
        
        // 4. Restore
        data->joy_rotation = save_rotation;
        data->joy_position = save_position;

        return ok;
    }

    bool BODY_ZY_FIX_LEGS_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        assert(data);

        if(!data->joystick_ok)
            return false;

        // 1. Variables
        Eigen::Quaterniond save_rotation(data->joy_rotation);
        Eigen::Vector3d save_position = data->joy_position;
        
        Eigen::Quaterniond rotation(data->joy_rotation);
        Eigen::Vector3d& position = data->joy_position;

        // 2. Clamp to zy
        position[0] = 0;
        position[1] = 0;
        rotation.x() = 0;
        rotation.y() = 0;
        
        data->joy_rotation = rotation;

        // 3. Run
        bool ok = XYZRPY_FIX_LEGS.run(target, data);
        
        // 4. Restore
        data->joy_rotation = save_rotation;
        data->joy_position = save_position;

        return ok;
    }    

}
