#pragma once
#include <Eigen/Dense>


namespace control {
    struct control_data_t;
}

namespace teleop {

    class teleop_t {
    public:
        teleop_t() {}
        virtual ~teleop_t() {}
        
        // set up subscriptions
        //virtual void init();

        virtual bool get_teleop_data(control::control_data_t* data) = 0;
    };

}
