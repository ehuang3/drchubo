#pragma once
#include "control.h"

namespace control {
    
    class control_t;
    class control_data_t;

    class NULL_T : public control_t {
    public:
        NULL_T(std::string name) : control_t(name) {}
        ~NULL_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
    };

}
