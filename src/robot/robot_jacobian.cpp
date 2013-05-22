#include "robot_jacobian.h"
#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
//#define NDEBUG
#include <assert.h>
#define DEBUG
#define MODULE_NAME "robot-jac"
#include "utils/debug_utils.h"

using namespace kinematics;
using namespace Eigen;
using namespace std;

namespace robot {

    robot_jacobian_t::robot_jacobian_t() {}

    robot_jacobian_t::~robot_jacobian_t() {}

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

    void robot_jacobian_t::find_dependent_dofs(vector<int>& dependent_dofs, BodyNode *end_effector)
    {
        dependent_dofs.clear();
        for(int i=0; i < end_effector->getNumDependentDofs(); ++i) {
            dependent_dofs.push_back(end_effector->getDependentDof(i));
        }
    }

    void robot_jacobian_t::manip_jacobian(MatrixXd& J, vector<int>& desired_dofs, 
                                          BodyNode *end_effector, const VectorXd& dofs)
    {
        // Build jacobian
        J.resize(6, end_effector->getNumDependentDofs());
        J.topRows(3) = end_effector->getJacobianLinear();
        J.bottomRows(3) = end_effector->getJacobianAngular();
        // Find dependent dofs
        vector<int> dependent_dofs;
        find_dependent_dofs(dependent_dofs, end_effector);
        // Intersect desired dofs with dependent dofs
        remap_jacobian(J, dependent_dofs, desired_dofs);
    }

    void robot_jacobian_t::remap_jacobian(MatrixXd& J, 
                                          const vector<int>& dependent_dofs,
                                          vector<int>& desired_dofs)
    {
        MatrixXd newJ(J.rows(), desired_dofs.size());
        newJ.setZero();
        vector<int> new_order;
        for(int i=0; i < dependent_dofs.size(); ++i) {
            bool found = false;
            for(int j=0; j < desired_dofs.size(); ++j) {
                if(dependent_dofs[i] == desired_dofs[j]) {
                    found = true;
                    break;
                }
            }
            if(found) {
                int k = new_order.size();
                newJ.col(k) = J.col(i);
                new_order.push_back(dependent_dofs[i]);
            }
        }
        J = newJ;
        desired_dofs = new_order;

        assert(J.cols() == desired_dofs.size());
    }

}
