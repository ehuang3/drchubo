#pragma once
#include "teleop.h"
#include <vector>
#include <Eigen/Dense>

// ROS stuff
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/Joy.h>

namespace Eigen {
    typedef Matrix<double,6,1> Vector6d;
}

namespace teleop {

    class spnav_t : public teleop_t {
    public:
		spnav_t() {}
		virtual ~spnav_t() {}

        virtual bool calibrate(const sensor_msgs::Joy::ConstPtr& _j);
		bool sensor_update(const sensor_msgs::Joy::ConstPtr& _j);
        virtual bool get_teleop_data(control::control_data_t* data);

        //############################################################
        //### ROS fields
        //############################################################
        // figure this out later
        ros::Time lastTime;
        
        //############################################################
        //### Initialization fields
        //############################################################
        static const int CALIBRATION_MAX = 50;
        Eigen::Vector6d axes_thresh;
        
        //############################################################
        //### Internal states
        //############################################################
        std::vector<float> last_axes;
        std::vector<int> last_buttons;

        //############################################################
        //### Teleoperation data
        //############################################################
        std::vector<int> buttons; //< buttons that are depressed
        std::vector<int> buttons_triggered; //< buttons that have been toggled
        
        Eigen::VectorXd joy_raw; //< raw joystick axes
        Eigen::VectorXd joy_filtered; //< filtered joystick
        Eigen::VectorXd joy_movement; //< filtered and time derivated joystick

        Eigen::Matrix3d joy_rotation;
        Eigen::Vector3d joy_position;

        bool joystick_ok; //< signals if input is ready for use
        
    };

}
