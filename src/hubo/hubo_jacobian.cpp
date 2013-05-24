#include "hubo_jacobian.h"
#include <kinematics/Skeleton.h>

namespace hubo {

    void hubo_jacobian_t::init(kinematics::Skeleton* _hubo) {
        robot = _hubo;
    }

}
