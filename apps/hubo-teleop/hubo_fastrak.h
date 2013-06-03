#pragma once
#include <robot/robot_state.h>
#include <control/control_data.h>
#include "Fastrak.h"


class fastrak_t {
public:
    fastrak_t();
    ~fastrak_t() {}

    void calibrate(robot::robot_state_t& robot, control::control_data_t* data);
    void update_sensors(control::control_data_t* data);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
    std::vector<int> hand_index;
    std::vector<int> sensor_index;

    Eigen::Matrix3d hand_orient;

    Eigen::Isometry3d raw_frame[2];
    Fastrak fastrak;
};
