#pragma once
#include "control.h"

namespace control {

    class control_t;
    class control_data_t;

    //############################################################
    //### Arm: Analytical IK
    //############################################################
    class ARM_AIK_T : public control_t {
    public:
    	ARM_AIK_T(std::string name) : control_t(name) {}
        ~ARM_AIK_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
    };
    
    //############################################################
    //### Arm: Analytical + Jacobian IK
    //############################################################
    class ARM_AJIK_T : public control_t {
    public:
        ARM_AJIK_T(std::string name) : control_t(name) {}
        ~ARM_AJIK_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
    };

    //############################################################
    //### Arm: Iterative Jacobian - Position + Orientation
    //############################################################
    class ARM_JIT_T : public control_t {
    public:
        ARM_JIT_T(std::string name) : control_t(name) {}
        ~ARM_JIT_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
    };

    class ARM_BOTH_JIT_T : public control_t {
    public:
    	ARM_BOTH_JIT_T(std::string name) : control_t(name) {}
        ~ARM_BOTH_JIT_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
    };
    
    //############################################################
    //### Arm: Iterative Jacobian - Position
    //############################################################
    class ARM_JPIT_T : public control_t {
    public:
        ARM_JPIT_T(std::string name) : control_t(name) {}
        ~ARM_JPIT_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
    };

    //############################################################
    //### Arm: Jacobian IK - Position + Orientation
    //############################################################
    class ARM_JIK_T : public control_t {
    public:
        ARM_JIK_T(std::string name) : control_t(name) {}
        ~ARM_JIK_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
    };
    
    //############################################################
    //### Arm: Jacobian IK - Position
    //############################################################
    class ARM_JPIK_T : public control_t {
    public:
        ARM_JPIK_T(std::string name) : control_t(name) {}
        ~ARM_JPIK_T() {}
        virtual bool run(robot::robot_state_t& target, control_data_t* data);
    };

}
