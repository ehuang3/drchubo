#pragma once
#include <robot/robot_state.h>

namespace atlas {

    class atlas_state_t : public robot::robot_state_t {
    public:
        virtual void init(kinematics::Skeleton *_robot);
    };

}
