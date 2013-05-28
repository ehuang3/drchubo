#include "atlas_jacobian.h"
#include "robot/robot.h"
#include <kinematics/BodyNode.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Joint.h>
#define DEBUG
#define MODULE_NAME "atlas-jacobians"
#include "utils/debug_utils.h"

using namespace robot;
using namespace kinematics;

namespace atlas {

    void atlas_jacobian_t::init(Skeleton *_atlas) {
        robot = _atlas;
    }

}
