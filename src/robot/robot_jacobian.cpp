#include "robot_jacobian.h"
#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#define DEBUG
#define MODULE_NAME "robot-jac"
#include "utils/debug_utils.h"

using namespace Eigen;
using namespace std;

namespace robot {

    robot_jacobian_t::robot_jacobian_t() {}

    robot_jacobian_t::~robot_jacobian_t() {}

    void robot_jacobian_t::arm_jacobian(MatrixXd& J, bool left, const VectorXd& dofs) {
        manip_jacobian(J, left ? MANIP_L_HAND : MANIP_R_HAND, dofs);
    }

    void robot_jacobian_t::leg_jacobian(MatrixXd& J, bool left, const VectorXd& dofs) {
        manip_jacobian(J, left ? MANIP_L_FOOT : MANIP_R_FOOT, dofs);
    }

    void robot_jacobian_t::manip_jacobian(MatrixXd& J, ManipIndex mi, const VectorXd& dofs) {
        int num_q = manip_joints[mi].size();
        robot->setPose(dofs, true);
        J.resize(6, num_q);

        MatrixXd jlin = robot->getNode(manip_node[mi].c_str())->getJacobianLinear();
        MatrixXd jang = robot->getNode(manip_node[mi].c_str())->getJacobianAngular();

        DEBUG_PRINT("num q = %d\n", num_q);

        DEBUG_STREAM << "jac lin = \n" << jlin << endl;
        DEBUG_STREAM << "jac ang = \n" << jang << endl;
        
        J.block(0, 0, 3, num_q) = robot->getNode(manip_node[mi].c_str())->getJacobianLinear().block(0, 0, 3, num_q);
        J.block(3, 0, 3, num_q) = robot->getNode(manip_node[mi].c_str())->getJacobianAngular().block(0, 0, 3, num_q);
    }

    void robot_jacobian_t::get_indexes(VectorXd& indexes, ManipIndex mi) {
        DEBUG_PRINT("Getting indices ... \n");
        indexes.resize(manip_joints[mi].size());
        for(int i=0; i < manip_joints[mi].size(); i++) {
            indexes(i) = robot->getJoint(manip_joints[mi][i].c_str())->getDof(0)->getSkelIndex();
            DEBUG_PRINT("joint %s\n", robot->getDof(indexes(i))->getJoint()->getName());
        }
    }

    void robot_jacobian_t::get_jacobian(MatrixXd& J, const MatrixXd& indexes, const VectorXd& dofs) {
        // Set robot pose
        robot->setPose(dofs);
        // Resize J to correct r c
        int rows = indexes.rows();
        int cols = indexes.cols();
        J.resize(rows*6, cols);
        // 
        
    }

}
