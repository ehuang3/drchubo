#include "robot_jacobian.h"
#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
//#define NDEBUG
#include <assert.h>
//#define DEBUG
#define MODULE_NAME "robot-jac"
#include "utils/debug_utils.h"

#include "robot_state.h"
#include <amino.h>



using namespace kinematics;
using namespace Eigen;
using namespace std;

namespace robot {

    robot_jacobian_t::robot_jacobian_t() {}

    robot_jacobian_t::~robot_jacobian_t() {}

    void robot_jacobian_t::manip_jacobian_ik(const Isometry3d& B, vector<int>& desired_dofs,
                                             BodyNode *base_frame, BodyNode *end_effector,
                                             robot_state_t& state)
    {

        // Avoid scenario where body node ptrs are from a different skeleton
        if(state.robot() != base_frame->getSkel() || state.robot() != end_effector->getSkel()) {
            ERROR_PRINT("Body node and state skeletons disagree (!=)!\n");
            state.dofs() = base_frame->getSkel()->getPose();
            base_frame = state.robot()->getNode(base_frame->getName());
            end_effector = state.robot()->getNode(end_effector->getName());
        }
        
        Skeleton *robotSkel = state.robot();
        robotSkel->setPose(state.dart_pose());

        // Loop parameters
        double tol = 1e-5;
        double alpha = 1;
        int max_iter = 1e3;
        // Jacobian variables
        MatrixXd J;
        VectorXd q(desired_dofs.size());
        state.get_dofs(q, desired_dofs);
        VectorXd qdot(desired_dofs.size());
        // Error variables
        Vector6d error;
        // wouidl like this const
        Isometry3d Twf; //< xform world to foot (the "base" frame)
        Isometry3d Tfw;
        Twf = base_frame->getWorldTransform();
        Isometry3d Tbf; //< xform B to foot
        Tbf = B.inverse() * Twf;
        Isometry3d Twe; //< xform world to end effector
        Isometry3d Twr; //< xform world to root (body) frame for repositioning
        // Main loop
        int i=0;
        while(i++ < max_iter) {
            // DEBUG_PRINT("iter %d\n", i);

            // Map error to foot frame
            Twe = end_effector->getWorldTransform();
            xform_error(error, Twe*Tbf, Twf);

            if(error.norm() < tol) {
                break;
            }

            error *= alpha;

            manip_jacobian(J, desired_dofs, base_frame, state);

            aa_la_dls(J.rows(), J.cols(), 0.05, J.data(), error.data(), qdot.data());

            q += qdot;
            state.set_dofs(q, desired_dofs);

            // Reposition foot frame to original location
            robotSkel->setPose(state.dart_pose());
            
            Tfw = base_frame->getWorldTransform();
            Tfw = Tfw.inverse();
            state.get_body(Twr);
            Twr = Twf * Tfw * Twr;
            state.set_body(Twr);
            
            robotSkel->setPose(state.dart_pose());
        }
        
    }

    void robot_jacobian_t::manip_jacobian_ik(const Isometry3d& B, vector<int>& desired_dofs,
                                             BodyNode *end_effector, robot_state_t& state)
    {

        double tol = 1e-5;
        double alpha = 1;
        int max_iter = 1e3;
        //
        MatrixXd J;
        VectorXd q(desired_dofs.size());
        state.get_dofs(q, desired_dofs);
        VectorXd qdot(desired_dofs.size());
        // 
        Vector6d error;
        Isometry3d A;
        //
        int i=0;
        while(i++ < max_iter) {
            // DEBUG_PRINT("iter %d\n", i);

            end_effector->getSkel()->setPose(state.dofs());

            A = end_effector->getWorldTransform();

            xform_error(error, B, A);
            
            // ERROR_PRINT("%d error %.16f\n", i, error.norm());

            // DEBUG_PRINT("error norm %f\n", error.norm());
            // DEBUG_STREAM << "error = \n" << error << endl;

            if(error.norm() < tol) {
                break;
            }

            error *= alpha;

            manip_jacobian(J, desired_dofs, end_effector, state);

            // DEBUG_STREAM << "jac = \n" << J << endl;

            aa_la_dls(J.rows(), J.cols(), 0.05, J.data(), error.data(), qdot.data());

            q += qdot;
            state.set_dofs(q, desired_dofs);
            state.clamp_indexes(desired_dofs, false);
        }
        
    }

    void robot_jacobian_t::xform_error(Vector6d& error, const Isometry3d& B, const Isometry3d& A) {
        AngleAxisd aa;
        Vector3d r;
        aa.fromRotationMatrix((A.inverse()*B).rotation());
        r = aa.axis() * aa.angle();
        r = A.rotation()*r;
        error.block<3,1>(0,0) = B.translation() - A.translation();        
        error.block<3,1>(3,0) = r;
    }

    void robot_jacobian_t::find_dependent_dofs(vector<int>& dependent_dofs, BodyNode *end_effector)
    {
        dependent_dofs.clear();
        for(int i=0; i < end_effector->getNumDependentDofs(); ++i) {
            dependent_dofs.push_back(end_effector->getDependentDof(i));
        }
    }

    void robot_jacobian_t::manip_jacobian(MatrixXd& J, vector<int>& desired_dofs, 
                                          BodyNode *end_effector, robot_state_t& state)
    {
        // Set dofs!
        end_effector->getSkel()->setPose(state.dofs(), true); //< it's kinda weird...
        // Build jacobian
        J.resize(6, end_effector->getNumDependentDofs());
        J.topRows(3) = end_effector->getJacobianLinear();
        J.bottomRows(3) = end_effector->getJacobianAngular();
        // Find dependent dofs
        vector<int> dependent_dofs;
        find_dependent_dofs(dependent_dofs, end_effector);
        // Intersect desired dofs with dependent dofs
        remap_jacobian(J, dependent_dofs, desired_dofs);
        // Clamp joints at limits
        clamp_jacobian(J, desired_dofs, state);
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

    void robot_jacobian_t::clamp_jacobian(MatrixXd& J, const vector<int>& desired_dofs,
                                          robot_state_t& state)
    {
        // double eps = 1e-3;
        for(int i=0; i < desired_dofs.size(); ++i) {
            double d = state.dofs(i);
            if( d < state.robot()->getDof(desired_dofs[i])->getMin() ||
                d > state.robot()->getDof(desired_dofs[i])->getMax() ) {
                J.col(i).setZero();
            }
        }
    }

}
