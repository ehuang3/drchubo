#include "legs.h"
#include "utils/robot_configs.h"
#include <iostream>
#include <assert.h>


namespace control {

    bool LEG_AIK_T::run(robot::robot_state_t& target, control_data_t* data)
    {
        assert(data);

        kinematics::Skeleton* robot = data->robot;
        robot::robot_kinematics_t* robot_kin = data->kin;
        int ms = data->manip_side;
        int mi = ms ? robot::MANIP_L_FOOT : robot::MANIP_R_FOOT;
        Eigen::Isometry3d Twfoot = data->manip_xform[mi];

        assert(robot);
        assert(robot_kin);
        assert(ms == 1 || ms == 0);

        Twfoot.linear() = Twfoot.linear() * data->joy_rotation;
        Twfoot.translation() += data->joy_position;

        bool ok = robot_kin->leg_ik(Twfoot, ms, target);

        assert(target.check_limits());

        data->manip_xform[mi] = Twfoot;
        data->manip_target = Twfoot;

        return ok;
    }

}
