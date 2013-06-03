#include "hubo_fastrak.h"
#include "Fastrak.h"
#include <iostream>
#include <stdio.h>

fastrak_t::fastrak_t()
{
    hand_index = { robot::MANIP_R_HAND, robot::MANIP_L_HAND };
    sensor_index = { 0, 1 };
    hand_orient = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());

    reset_history();
}

void fastrak_t::reset_history() {
    hist = 0;
    for(int j=0; j < 2; j++)
    for(int i=0; i < HISTORY_LENGTH; i++) {
        past_pos[i][j] = Eigen::Vector3d::Zero();
        past_quat[i][j] = Eigen::Matrix3d::Identity();
    }
}

void fastrak_t::update_average(Eigen::Isometry3d& B, int ms)
{
    int p = (hist++)%HISTORY_LENGTH;
    
    past_pos[p][ms] = B.translation();
    past_quat[p][ms] = B.linear();

    Eigen::Vector3d avg_pos = past_pos[0][ms];

    Eigen::Vector3d avg_quat_vec = past_quat[0][ms].vec();
    double avg_quat_w = past_quat[0][ms].w();


    for(int i=1; i < HISTORY_LENGTH; i++) {
        avg_pos = avg_pos + past_pos[i][ms];
        avg_quat_vec += past_quat[i][ms].vec();
        avg_quat_w += past_quat[i][ms].w();
    }

    avg_pos /= HISTORY_LENGTH;
    
    Eigen::Quaterniond avg_quat;
    avg_quat.w() = avg_quat_w/HISTORY_LENGTH;
    avg_quat.vec() = avg_quat_vec;
    avg_quat.normalize();

    B.translation() = avg_pos;
    B.linear() = avg_quat.matrix();
}

void fastrak_t::calibrate(robot::robot_state_t& target, control::control_data_t* data)
{
    std::cout << "[fastrak_t] Calibrating...\n";
    
    // Fill nominal frame of sensors
    for(int i=0; i < sensor_index.size(); i++) {
        fastrak.getPose(raw_frame[i], sensor_index[i], true);
    }

    reset_history();
}

void fastrak_t::update_sensors(control::control_data_t* data)
{
    for(int i=0; i < hand_index.size(); i++) {
        Eigen::Isometry3d tf;
        fastrak.getPose(tf, sensor_index[i], true);

        update_average(tf, i);

        tf = raw_frame[i].inverse() * tf;

        data->sensor_tf[i] = tf;
    }
    data->sensor_ok = true;
}
