#pragma once
#include "control.h"

namespace control {

    class control_t;
    class control_data_t;

    //############################################################
    //### Leg: Analytical IK
    //############################################################
    class JOINT_T : public control_t {
    public:
    	JOINT_T(std::string name) : control_t(name), joint_index(0), joint_changed(true) {}
        ~JOINT_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
        virtual bool change_mode(int ch, robot::robot_state_t& target);
        int joint_index;
        bool joint_changed;
    };

}
