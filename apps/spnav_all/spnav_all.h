#pragma once

enum Teleop_Mode {
    TELEOP_COM,
    TELEOP_LEFT_ARM,
    TELEOP_RIGHT_ARM,
    NUM_TELEOP_MODES,
    TELEOP_LEFT_LEG,
    TELEOP_RIGHT_LEG,
    TELEOP_BACK,
    TELEOP_HEAD,
    TELEOP_LEFT_HAND,
    TELEOP_RIGHT_HAND,
    TELEOP_JOINT,
};

namespace atlas {
    class atlas_state_rosified : public atlas_state_t {
    public:
        void fill_joint_command(atlas_msgs::AtlasCommand* jointCommand,
                                ros::NodeHandle* rosnode);
    };
}
