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
    
        virtual void init(kinematics::Skeleton *_robot) { robot = _robot; }

        // function: manip_jacobian(J, desired_dofs, end_effector, state)
        // brief: Computes the Jacobian to the end effector restricted to the desired
        //        dofs.
        // input: J - The matrix to write the Jacobian into.
        //        desired_dofs - The Jacobian is restricted to these joints.
        //        end_effector - The target link.
        //        state - The current pose of the robot.
        void manip_jacobian(Eigen::MatrixXd& J, std::vector<int>& desired_dofs,
                            kinematics::BodyNode *end_effector, robot_state_t& state);

        // function: remap_jacobian(J, dependent_dofs, desired_dofs)
        // brief: Truncates J to only the joints in the intersection of dependent dofs
        //        and desired dofs.
        // caveat: Will reorder desired_dofs to agree with J.
        void remap_jacobian(Eigen::MatrixXd& J, const std::vector<int>& dependent_dofs, 
                            std::vector<int>& desired_dofs);

        // function: clamp_jacobian(J, desired_dofs, state)
        // brief: Zeros the columns of J corresponding to joints at their limits.
        // input: J - The jacobian.
        //        desired_dofs - Desired joints to control.
        //        state - Current pose of the robot.
        // caveat: Assumes J is ordered the same way as desired dofs.
        void clamp_jacobian(Eigen::MatrixXd& J, const std::vector<int>& desired_dofs,
                            robot_state_t& state);

        // function: find_dependent_dofs(dependent_dofs, end_effector)
        // brief: Truncates the vector of dependent dofs to only those along the chain from the
        //        end effector to the root node.
        // input: dependent_dofs - Joint indexes corresponding to the DART dofs of interest.
        //        end_effector -  The target link.
        // caveat: The root node is somewhat arbitrarily placed, so retrieving the chain from hand
        //         to foot is tricky (hence unimplemented).
        void find_dependent_dofs(std::vector<int>& dependent_dofs, kinematics::BodyNode *end_effector);

        // function: xform_error(err, Twb, Twa)
        // brief: Computes the translation and rotation error required to move from Twa to Twb.
        //        The error is in the format that the manipulator jacobian expects, which is DART
        //        format. DART format is xyz error, axis and maginitude required to rotate from Twa
        //        to Twb.
        void xform_error(Eigen::Vector6d& err, const Eigen::Isometry3d& Twb, const Eigen::Isometry3d& Twa);

        // WARNING: Not tested.
        // function: manip_jacobian_ik(B, desired_dofs, base_frame, end_effector, state)
        // brief: Attempts to do IK on the end effector from the base frame.
        void manip_jacobian_ik(const Eigen::Isometry3d& B, std::vector<int>& desired_dofs,
                               kinematics::BodyNode *base_frame, kinematics::BodyNode *end_effector,
                               robot_state_t& state);
        // WARNING: Not tested.
        // brief: Attempts to do IK to move the end effector onto the input frame B.
        void manip_jacobian_ik(const Eigen::Isometry3d& B, std::vector<int>& desired_dofs,
                               kinematics::BodyNode *end_effector, robot_state_t& state);

    protected:
        kinematics::Skeleton *robot;
    };

}
