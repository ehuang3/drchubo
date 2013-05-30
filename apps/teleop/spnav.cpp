#include "spnav.h"
#include <stdio.h>
#include <control/control_data.h>

// ROS stuff
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/Joy.h>

namespace teleop {

    bool spnav_t::calibrate(const sensor_msgs::Joy::ConstPtr& _j)
    {
        //############################################################
        //### Calibration
        //############################################################
        static int calibration_count = 0;
        if(calibration_count >= CALIBRATION_MAX)
            return false;
        if(calibration_count == 0) {
            std::cout << "[spnav] Calibrating Space Nav\n";
            axes_thresh = Eigen::Vector6d::Zero();
        }
        calibration_count++;
        
        for(int i=0; i < 6; i++)
            if(fabs(_j->axes[i]) > axes_thresh(i))
                axes_thresh(i) = fabs(_j->axes[i]);

        if(calibration_count == CALIBRATION_MAX) {
            if(axes_thresh.norm() < 0.01) {
                printf("[spnav] No error ... faking thresholds\n");
                axes_thresh = 0.02 * Eigen::Vector6d::Ones();
            }
            std::cout << "[spnav] Thresholds = " << axes_thresh.transpose() << std::endl;
            std::cout << "[spnav] Thresh norm = " << axes_thresh.norm() << std::endl;            
        }

        return true;
    }

    bool spnav_t::sensor_update(const sensor_msgs::Joy::ConstPtr& _j) {
        //############################################################
        //### ROS stuff
        //############################################################
        static ros::Time lastTime = _j->header.stamp;
        ros::Time currentTime = _j->header.stamp;
        ros::Duration dT = currentTime - lastTime;
        
        //############################################################
        //### Calibrate joystick
        //############################################################
        // 1. Do calibration before proceeding
        if(calibrate(_j))
            return false;

        //############################################################
        //### Filter joy stick inputs
        //############################################################
        // 1. First, threshold the joystick using the threshold norm
        double thresh = 2 * axes_thresh.norm(); // Add slack, as joystick tends to not zero properly
        int max_scale = 2; // scale threshold for maxmimum input velocity
        // Zero out commands
        joy_raw = Eigen::Vector6d::Zero();
        joy_filtered = Eigen::Vector6d::Zero();
        joy_movement = Eigen::Vector6d::Zero();
        // Threshold the commands
        bool allBelow = true;
        for(int i=0; i < 6; i++) {
            joy_raw(i) = _j->axes[i];
            if( fabs(_j->axes[i]) <= thresh )
                continue;
            joy_filtered[i] = _j->axes[i];
            if( fabs(joy_filtered[i]) > max_scale * thresh )
                joy_filtered[i] *= max_scale*thresh / fabs(joy_filtered[i]);
            allBelow = false;
        }
        // 2. Convert to move speeds 
        double movespeed = .00005;
        joy_movement.resize(6); // the desired end effector velocity in worldspace
        joy_movement[0] = movespeed * joy_filtered[0] / dT.toSec(); // translate +x
        joy_movement[1] = movespeed * joy_filtered[1] / dT.toSec(); // translate +y
        joy_movement[2] = movespeed * joy_filtered[2] / dT.toSec(); // translate +z
        joy_movement[3] = movespeed * joy_filtered[3] / dT.toSec(); // rotate + around x
        joy_movement[4] = movespeed * joy_filtered[4] / dT.toSec(); // rotate + around y
        joy_movement[5] = movespeed * joy_filtered[5] / dT.toSec(); // rotate + around z
        // 3. Sanity check
        bool invalid_input = dT.toSec() == 0;
        for(int i=0; i < 6; i++) {
            invalid_input |= isnan(joy_movement[0]);
            invalid_input |= isinf(joy_movement[0]);
        }
        // 4. Are we ok?
        joystick_ok = !invalid_input && !allBelow;

        //############################################################
        //### Convert to transformations
        //############################################################
        // 1. Be optimistic
        sensor_ok = true;
        // 2. Offset
        sensor_position = joy_movement.block<3,1>(0,0); //< 
        // 3. Rotation
        Eigen::Vector3d r_axes = joy_filtered.block<3,1>(3,0).normalized();
        Eigen::Vector3d r_movement = joy_movement.block<3,1>(3,0); //< DO NOT USE THIS FOR AXES
        double r_omega = r_movement.norm();
        // 4. Filtered rotation axes are possible 0 ==> nan values after normalization
        if(isnan(r_axes[0]))
            sensor_ok = false;
        else
            sensor_rotation = Eigen::AngleAxisd(r_omega, r_axes);

        //############################################################
        //### Button updates
        //############################################################
        // 2. Check if flip flop
        buttons = _j->buttons;
        buttons_triggered = buttons; //< just to initialize buttons_triggered
        for(int i=0; i < buttons.size(); i++) {
            buttons_triggered[i] = 0;
            if(buttons[i] && !last_buttons[i])
                buttons_triggered[i] = 1;
        }
        // 3. Remember the past
        last_buttons = _j->buttons;
        last_axes = _j->axes;
        lastTime = currentTime;

        return true;
    }
    
    bool spnav_t::get_teleop_data(control::control_data_t* data) {
        data->buttons.resize(buttons.size());
        data->triggers.resize(buttons_triggered.size());
        for(int i=0; i < buttons.size(); i++) {
            data->buttons[i] = buttons[i];
            data->triggers[i] = buttons_triggered[i];
        }
        data->joy_raw = joy_raw;
        data->joy_filtered = joy_filtered;
        data->joy_movement = joy_movement;
        data->joystick_ok = joystick_ok;
        data->sensor_position = sensor_position;
        data->sensor_rotation = sensor_rotation;
        data->sensor_ok = sensor_ok;
        return true;
    }

}
