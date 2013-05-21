#pragma once
#include "robot.h"
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace kinematics { class Skeleton; }

namespace robot {

    class robot_jacobian_t {
    public:
        robot_jacobian_t();
        virtual ~robot_jacobian_t();
    
        virtual void init(kinematics::Skeleton *_robot) = 0;
        
        void arm_jacobian(Eigen::MatrixXd& J, bool side, const Eigen::VectorXd& dofs);
        void leg_jacobian(Eigen::MatrixXd& J, bool side, const Eigen::VectorXd& dofs);
        void manip_jacobian(Eigen::MatrixXd& J, ManipIndex mi, const Eigen::VectorXd& dofs);
        
        void get_indexes(Eigen::MatrixXd& indexes, const std::string& base, 
                         const std::vector<std::string>& end_effectors);
        
        void get_indexes(Eigen::VectorXd& indexes, ManipIndex mi);
        /* void get_indexes(Eigen::VectorXd& indexes, BodyNode *end, BodyNode *base); */

        std::string& name(ManipIndex mi) { return manip_node[mi]; }
    
        void get_jacobian(Eigen::MatrixXd& J, const Eigen::MatrixXd& indexes, const Eigen::VectorXd& dofs);
    
    protected:
        kinematics::Skeleton *robot;
        std::string manip_node[NUM_MANIPULATORS];
        std::vector<std::string> manip_joints[NUM_MANIPULATORS];
    };

}
