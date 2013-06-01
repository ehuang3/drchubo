#pragma once
#include "control.h"

namespace control {
    
    class control_t;
    class control_data_t;

    //############################################################
    //### BODY: XYZ FIX LEGS
    //############################################################
    class BODY_XYZRPY_FIX_LEGS_T : public control_t {
    public:
    	BODY_XYZRPY_FIX_LEGS_T(std::string name) : control_t(name) {}
        ~BODY_XYZRPY_FIX_LEGS_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
    };

    //############################################################
    //### BODY: XZ-P FIX LEGS
    //############################################################
    class BODY_XZP_FIX_LEGS_T : public control_t {
    public:
    	BODY_XZP_FIX_LEGS_T(std::string name) : control_t(name), xyzrpy_fix_legs("")  {}
        ~BODY_XZP_FIX_LEGS_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
        BODY_XYZRPY_FIX_LEGS_T xyzrpy_fix_legs;
    };

    //############################################################
    //### BODY: Z-Y FIX LEGS
    //############################################################
    class BODY_ZY_FIX_LEGS_T : public control_t {
    public:
    	BODY_ZY_FIX_LEGS_T(std::string name) : control_t(name), XYZRPY_FIX_LEGS("") {}
        ~BODY_ZY_FIX_LEGS_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
        BODY_XYZRPY_FIX_LEGS_T XYZRPY_FIX_LEGS;
    };

}
