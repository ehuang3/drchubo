#pragma once
#include "robot/robot_jacobian.h"

namespace atlas {

class atlas_jacobian_t : public robot::robot_jacobian_t {
public:
    atlas_jacobian_t() {};
    virtual ~atlas_jacobian_t() {};

    void init(kinematics::Skeleton *_atlas);
};

}
