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

    // History buffer of past positions and rotations
#define HISTORY_LENGTH 15
    int hist;
    Eigen::Vector3d past_pos[HISTORY_LENGTH][2];
    Eigen::Quaterniond past_quat[HISTORY_LENGTH][2];

    void reset_history();
    void update_average(Eigen::Isometry3d& B, int ms); //< returns new average taking B into account

    Fastrak fastrak;
};
