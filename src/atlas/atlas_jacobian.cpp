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
    DEBUG_PRINT("Initializing...\n");
    robot = _atlas;
    manip_node[MANIP_L_FOOT] = "l_foot";
    manip_node[MANIP_R_FOOT] = "r_foot";
    manip_node[MANIP_L_HAND] = "l_hand";
    manip_node[MANIP_R_HAND] = "r_hand";
    // Get manipulator joints
    for(int mi = 0; mi < NUM_MANIPULATORS; mi++) {
        Joint *joint = _atlas->getNode(manip_node[mi].c_str())->getParentJoint();
        DEBUG_PRINT("Manipulator: %s\n", manip_node[mi].c_str());
        DEBUG_PRINT("Kinematic Chain:\n");
        for(int ji = 0; ji < 6; ji++) {
            manip_joints[mi].push_back(joint->getName());
            DEBUG_PRINT("%d %s\n", ji, manip_joints[mi][ji].c_str());
            joint = joint->getParentNode()->getParentJoint();
        }
    }
}

}
