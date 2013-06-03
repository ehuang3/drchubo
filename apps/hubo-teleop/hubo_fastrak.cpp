#include "hubo_fastrak.h"
#include "Fastrak.h"
#include <iostream>
#include <stdio.h>

fastrak_t::fastrak_t()
{
    hand_index = { robot::MANIP_R_HAND, robot::MANIP_L_HAND };
    sensor_index = { 0, 1 };
    hand_orient = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
}

void fastrak_t::calibrate(robot::robot_state_t& target, control::control_data_t* data)
{
    std::cout << "[fastrak_t] Calibrating...\n";
    
    // Fill nominal frame of sensors
    for(int i=0; i < sensor_index.size(); i++) {
        fastrak.getPose(raw_frame[i], sensor_index[i], true);
    }
}

void fastrak_t::update_sensors(control::control_data_t* data)
{
    for(int i=0; i < hand_index.size(); i++) {
        Eigen::Isometry3d tf;
        fastrak.getPose(tf, sensor_index[i], true);

        tf = raw_frame[i].inverse() * tf;

        data->sensor_tf[i] = tf;
    }
    data->sensor_ok = true;
}
