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
    
    // Fill nominal frame of hands
    for(int i=0; i < hand_index.size(); i++) {
        hand[i] = data->manip_xform[ hand_index[i] ];

        hand[i].linear() = hand[i].linear() * hand_orient;
    }

    std::cout << "World left hand = \n" << hand[1].matrix() << std::endl;

    // Fill nominal frame of sensors
    for(int i=0; i < sensor_index.size(); i++) {
        fastrak.getPose(raw_frame[i], sensor_index[i], true);
    }

    std::cout << "Fastrak left hand = \n" << raw_frame[1].matrix() << std::endl;
}

void fastrak_t::update_sensors(control::control_data_t* data)
{
    std::cout << "[fastrak_t] Updating ...\n";

    for(int i=0; i < hand_index.size(); i++) {
        Eigen::Isometry3d tf;
        fastrak.getPose(tf, sensor_index[i], true);

        Eigen::Matrix3d dR = raw_frame[i].rotation().inverse() * tf.rotation();
        Eigen::Vector3d dv = (raw_frame[i].inverse() * tf).translation();

        std::cout << "Delta rotation left hand = \n" << dR << std::endl;
        std::cout << "Delta translation left hand = " << dv.transpose() << std::endl;

        tf = hand[i] * raw_frame[i].inverse() * tf;

        tf.linear() = tf.linear() * hand_orient.inverse();

        data->sensor_tf[i] = tf;
    }
    data->sensor_ok = true;
}
