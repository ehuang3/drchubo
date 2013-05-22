#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <atlas/atlas_kinematics.h>
#include <utils/math_utils.h>
#include <utils/data_paths.h>
#include <math/EigenHelper.h>

#include <math.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Transformation.h>
#include <dynamics/SkeletonDynamics.h>

#include <amino.h>

#include <atlas/atlas_jacobian.h>

using namespace std;
using namespace Eigen;
using namespace atlas;
using namespace robot;

using namespace kinematics;
using namespace dynamics;
using namespace simulation;
/* ********************************************************************************************* */
#define ROBOT_URDF "models/atlas/atlas_world.urdf"
#define ROBOT_NAME "atlas"
#define LEFT_HAND "l_hand"
#define RIGHT_HAND "r_hand"
#define ROBOT_JACOBIAN_T atlas::atlas_jacobian_t
/* ********************************************************************************************* */
kinematics::Skeleton* _robot_;
Skeleton* PREPARE_ROBOT() {
    if(_robot_ == 0) {
        DartLoader dart_loader;
		World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH ROBOT_URDF);
		_robot_ = mWorld->getSkeleton(ROBOT_NAME);
    }
    return _robot_;
}
/* ********************************************************************************************* */
ROBOT_JACOBIAN_T* _rj_;
robot_jacobian_t* PREPARE_ROBOT_JACOBIAN() {
    if(_rj_ == 0) {
        _rj_ = new ROBOT_JACOBIAN_T;
        _rj_->init(PREPARE_ROBOT());
    }
    return _rj_;
}
/* ********************************************************************************************* */
void DART_INCR_FK(Isometry3d& B, const VectorXd& q, ManipIndex mi) {
    robot_jacobian_t *rojac = PREPARE_ROBOT_JACOBIAN();
    Skeleton *robot = PREPARE_ROBOT();
    // Get and index in dofs
    VectorXd index;
    rojac->get_indexes(index, mi);
    VectorXd dofs = robot->getPose();
    for(int i=0; i < index.rows(); i++) {
        dofs(index(i)) += q(i);
    }
    robot->setPose(dofs, true);
    B = robot->getNode(rojac->name(mi).c_str())->getWorldTransform();
}
/* ********************************************************************************************* */
void PRINT_KINEMATIC_CHAIN(BodyNode *end_effector) {
    printf("%s kinematic chain:\n", end_effector->getName());
    Joint *parent = end_effector->getParentJoint();
    while(parent) {
        printf("%s\n", parent->getName());
        if(!parent->getParentNode())
            break;
        parent = parent->getParentNode()->getParentJoint();
    }
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_INIT) {
    PREPARE_ROBOT_JACOBIAN();
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_SINGLE_ARM) {
    Skeleton* robot = PREPARE_ROBOT();
    // Set dofs to something (?)
    VectorXd dofs = robot->getPose();
    dofs.setZero();
    robot->setPose(dofs, true);
    Isometry3d B;
    VectorXd q_zero(6);
    DART_INCR_FK(B, q_zero, MANIP_L_HAND);
    // Print out ground truth
    cout << "Tw_mwx = \n" << B.matrix() << endl;

    // Get manip jacobian
    robot_jacobian_t* rojac = PREPARE_ROBOT_JACOBIAN();
    MatrixXd jac;
    rojac->manip_jacobian(jac, MANIP_L_HAND, dofs);
    // Try damped least squares
    int m = jac.rows();
    int n = jac.cols();
    // Solve q for x
    VectorXd q(jac.cols());
    VectorXd x(6);
    x << 0, -0.1, 0, 0, 0, 0;
    aa_la_dls(m, n, 0.1, jac.data(), x.data(), q.data());
    // output
    cout << "dls q = \n" << q << endl;

    // See how we did
    dofs.setZero();
    robot->setPose(dofs, true);
    DART_INCR_FK(B, q, MANIP_L_HAND);

    cout << "Tw_mwx after jacobian = \n" << B.matrix() << endl;
        
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
