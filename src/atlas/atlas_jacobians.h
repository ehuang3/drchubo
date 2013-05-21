#pragma once
#include "robot/robot_jacobians.h"

namespace atlas {

class atlas_jacobians_t : public robot::robot_jacobians_t {
public:
    atlas_jacobians_t() {};
    virtual ~atlas_jacobians_t() {};

    void init(kinematics::Skeleton *_atlas);
};

}
