#pragma once
#include "control_data.h"
#include "control_factory.h"
#include <Eigen/Dense>
#include <string>

namespace control {

    class control_t {
    public:
        control_t(std::string name) { _name = name; }
        virtual ~control_t() {}
        
        virtual bool run(robot::robot_state_t& target, control_data_t* params) = 0;
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    private:
        Eigen::Isometry3d Tprev; //< for internal state
        std::string _name;
    };

}
