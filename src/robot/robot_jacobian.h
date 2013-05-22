#pragma once
#include "robot.h"
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace kinematics { class Skeleton; class BodyNode; }

namespace robot {
    
    class robot_state_t;

    class robot_jacobian_t {
    public:
        robot_jacobian_t();
        virtual ~robot_jacobian_t();
    
        virtual void init(kinematics::Skeleton *_robot) = 0;
        
        void get_indexes(Eigen::MatrixXd& indexes, const std::string& base, 
                         const std::vector<std::string>& end_effectors);
        
        void get_indexes(Eigen::VectorXd& indexes, ManipIndex mi);
        /* void get_indexes(Eigen::VectorXd& indexes, BodyNode *end, BodyNode *base); */

        std::string& name(ManipIndex mi) { return manip_node[mi]; }

        void get_jacobian(Eigen::MatrixXd& J, const Eigen::MatrixXd& indexes, const Eigen::VectorXd& dofs);

        void manip_jacobian_ik(const Eigen::Isometry3d& B, robot_state_t& r_state);

        void manip_jacobian(Eigen::MatrixXd& J, std::vector<int>& desired_dofs,
                            kinematics::BodyNode *end_effector, const Eigen::VectorXd& dofs);

        void remap_jacobian(Eigen::MatrixXd& J, const std::vector<int>& dependent_dofs, 
                            std::vector<int>& desired_dofs);

        void find_dependent_dofs(std::vector<int>& dependent_dofs, kinematics::BodyNode *end_effector);
    
    protected:
        kinematics::Skeleton *robot;
        std::string manip_node[NUM_MANIPULATORS];
        std::vector<std::string> manip_joints[NUM_MANIPULATORS];
    };

}
