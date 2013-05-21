#pragma once
#include "robot.h"
#include <Eigen/Dense>
#include <kinematics/Skeleton.h>
#include <string>
#include <vector>

namespace robot {

class robot_jacobians_t {
public:
    robot_jacobians_t();
    virtual ~robot_jacobians_t();

    virtual void init(kinematics::Skeleton *_robot) = 0;

    void armJacobian(Eigen::MatrixXd& J, int side);
    void legJacobian(Eigen::MatrixXd& J, int side);
    void manipJacobian(Eigen::MatrixXd& J, ManipIndex mi);

    void getIndexes(Eigen::MatrixXd& indexes, const std::string& base, 
                    const std::vector<std::string>& end_effectors);

    void getJacobian(Eigen::MatrixXd& J, const Eigen::MatrixXd& indexes, const Eigen::VectorXd& dofs);
        
protected:
    kinematics::Skeleton *robot;
    std::string manip_node[NUM_MANIPULATORS];
    std::vector<std::string> manip_joints[NUM_MANIPULATORS];

};

}
