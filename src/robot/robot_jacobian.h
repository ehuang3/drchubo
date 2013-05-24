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
        

        // void solve_dls(Eigen::MatrixXd& J, Eigen::VectorXd& err, Eigen::VectorXd& qdot);

        // Please use the functions below.

        void manip_jacobian_ik(Eigen::Isometry3d& B, std::vector<int>& desired_dofs,
                               kinematics::BodyNode *base_frame, kinematics::BodyNode *end_effector,
                               robot_state_t& state);

        void manip_jacobian_ik(Eigen::Isometry3d& B, std::vector<int>& desired_dofs,
                               kinematics::BodyNode *end_effector, robot_state_t& state);

        void manip_jacobian(Eigen::MatrixXd& J, std::vector<int>& desired_dofs,
                            kinematics::BodyNode *end_effector, robot_state_t& state);

        void remap_jacobian(Eigen::MatrixXd& J, const std::vector<int>& dependent_dofs, 
                            std::vector<int>& desired_dofs);

        void clamp_jacobian(Eigen::MatrixXd& J, const std::vector<int>& desired_dofs,
                            robot_state_t& state);

        void find_dependent_dofs(std::vector<int>& dependent_dofs, kinematics::BodyNode *end_effector);

        /* void hcat_jacobian(Eigen::MatrixXd& J1, const std::vector<int>& dd1, */
        /*                    Eigen::MatrixXd& J2, const std::vector<int>& dd2); */
       
        /* void vcat_jacobian(Eigen::MatrixXd& J1, const std::vector<int>& dd1, */
        /*                    Eigen::MatrixXd& J2, const std::vector<int>& dd2); */
        
        // returns error required to move from Twa to Twb
        // Twa - xfrom world to a
        void xform_error(Eigen::Vector6d& err, const Eigen::Isometry3d& Twb, const Eigen::Isometry3d& Twa);

    protected:
        kinematics::Skeleton *robot;
        std::string manip_node[NUM_MANIPULATORS];
        std::vector<std::string> manip_joints[NUM_MANIPULATORS];
    };

}
