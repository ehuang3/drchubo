#pragma once
#include "robot/robot_jacobian.h"

namespace hubo {

class hubo_jacobian_t : public robot::robot_jacobian_t {
public:
    hubo_jacobian_t() {};
    virtual ~hubo_jacobian_t() {};

    void init(kinematics::Skeleton *_atlas);
};

}
