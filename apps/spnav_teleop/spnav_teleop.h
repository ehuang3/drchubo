#pragma once


namespace atlas {
    class atlas_state_rosified : public atlas_state_t {
    public:
        void fill_joint_command(atlas_msgs::AtlasCommand* jointCommand,
                                ros::NodeHandle* rosnode);
    };
}
