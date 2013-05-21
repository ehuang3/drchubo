#pragma once

namespace robot {

    enum ManipIndex {
        MANIP_L_FOOT,
        MANIP_R_FOOT,
        MANIP_L_HAND,
        MANIP_R_HAND,
        NUM_MANIPULATORS,
    };
    
    enum IK_Mode {
        IK_MODE_FREE,
        IK_MODE_FIXED,
        IK_MODE_BODY,
        IK_MODE_WORLD,
        IK_MODE_SUPPORT,
    };

}
