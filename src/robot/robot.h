#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Eigen {
    typedef Matrix<double,6,1> Vector6d;
}

namespace robot {

    enum TeleopMode {
        TELEOP_COM,
        TELEOP_LEFT_ARM_JACOBIAN,
        TELEOP_RIGHT_ARM_JACOBIAN,
        TELEOP_LEFT_ARM_ANALYTIC,
        TELEOP_RIGHT_ARM_ANALYTIC,
        NUM_TELEOP_MODES,
        TELEOP_LEFT_LEG,
        TELEOP_RIGHT_LEG,
        TELEOP_BACK,
        TELEOP_HEAD,
        TELEOP_LEFT_HAND,
        TELEOP_RIGHT_HAND,
        TELEOP_JOINT,        
    };

    enum ManipIndex {
        MANIP_L_FOOT,
        MANIP_R_FOOT,
        MANIP_L_HAND,
        MANIP_R_HAND,
        NUM_MANIPULATORS,
    };

    enum LimbIndex {
        LIMB_L_LEG,
        LIMB_R_LEG,
        LIMB_L_ARM,
        LIMB_R_ARM,
        LIMB_HEAD,
        LIMB_TORSO,
        LIMB_L_HAND,
        LIMB_R_HAND,
        NUM_LIMBS,
    };
    
    enum IK_Mode {
        IK_MODE_FREE,
        IK_MODE_FIXED,
        IK_MODE_BODY,
        IK_MODE_WORLD,
        IK_MODE_SUPPORT,
    };

}
