#pragma once
#include "control.h"

namespace control {

    class control_t;
    class control_data_t;

    //############################################################
    //### Leg: Analytical IK
    //############################################################
    class LEG_AIK_T : public control_t {
    public:
    	LEG_AIK_T(std::string name) : control_t(name) {}
        ~LEG_AIK_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
    };

}
