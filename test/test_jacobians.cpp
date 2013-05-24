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
#include <robot/robot_state.h>
#include <atlas/atlas_state.h>

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
#define ROBOT_STATE_T atlas::atlas_state_t
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
robot_state_t* _rs_;
robot_state_t* PREPARE_ROBOT_STATE() {
    if(_rs_ == 0) {
        _rs_ = new ROBOT_STATE_T;
        _rs_->init(PREPARE_ROBOT());
    }
    return _rs_;
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
TEST(JACOBIAN, TEST_DART) {
    // Examine DART conventions for returning Jacobians
    Skeleton *robot = PREPARE_ROBOT();
    PRINT_KINEMATIC_CHAIN(robot->getNode(LEFT_HAND));
    BodyNode *end_effector = robot->getNode(LEFT_HAND);
    printf("dependent dofs:\n");
    for(int i=0; i < end_effector->getNumDependentDofs(); i++) {
        Dof *dof = robot->getDof(end_effector->getDependentDof(i));
        printf("%d %s %s\n", i, dof->getJoint()->getName(), dof->getName());
    }
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_REMAP_JACOBIAN) {
    robot_jacobian_t* rjac = PREPARE_ROBOT_JACOBIAN();
    
    MatrixXd J(3, 6);
    for(int i=0; i < J.cols(); i++)
        J.col(i) = i*Vector3d::Ones();
    
    vector<int> dependent = { 0, 1, 2, 3, 4, 5 };
    vector<int> desired = { 0, 5, 1 };
    
    rjac->remap_jacobian(J, dependent, desired);

    cout << "remapped J = \n" << J << endl;

}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_MANIP_JACOBIAN) {
    Skeleton *robot = PREPARE_ROBOT();
    robot_jacobian_t* rjac = PREPARE_ROBOT_JACOBIAN();
    robot_state_t* rstat = PREPARE_ROBOT_STATE();
    
    MatrixXd J;
    vector<int> desired_dofs;
    rstat->get_manip_indexes(desired_dofs, LIMB_L_ARM);
    BodyNode *l_arm = robot->getNode(LEFT_HAND);
    VectorXd dofs = robot->getPose();
    dofs.setZero();

    rstat->dofs() = dofs;

    rjac->manip_jacobian(J, desired_dofs, l_arm, *rstat);

    VectorXd qdot(J.cols());
    VectorXd x(6);
    x << 0, -0.01, 0, 0, 0, 0;
    
    aa_la_dls(J.rows(), J.cols(), 0.1, J.data(), x.data(), qdot.data());

    rstat->dart_pose() = dofs;
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_ANGLE_AXIS) {
    Isometry3d B;
    Isometry3d W;
    W = Matrix4d::Identity();
    B = Matrix4d::Identity();
    
    W.rotate(AngleAxisd(-M_PI/2, Vector3d::UnitX()));
    B.rotate(AngleAxisd(M_PI/2, Vector3d::UnitY()));

    Matrix3d R = B.rotation();

    AngleAxisd ob;
    ob.fromRotationMatrix(B.rotation());
    AngleAxisd ow;
    ow.fromRotationMatrix(W.rotation());

    cout << "ob = \n" << ob.axis() << endl;
    cout << "angle = \n" << ob.angle() << endl;

    cout << "ow = \n" << ow.axis() << endl;
    cout << "angle = " << ow.angle() << endl;

    AngleAxisd wb;
    wb.fromRotationMatrix(  (W.inverse() * B).rotation() );
   
    cout << "wb = \n" << wb.axis() << endl;
    cout << "angle = " << wb.angle() << endl;
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_MANIP_IK) {
    robot_jacobian_t* robot = PREPARE_ROBOT_JACOBIAN();
    robot_state_t* state = PREPARE_ROBOT_STATE();

    vector<int> desired_dofs;
    state->get_manip_indexes(desired_dofs, LIMB_L_ARM);

    BodyNode *left_hand = state->robot()->getNode(LEFT_HAND);
    state->dofs().setZero();

    VectorXd q(6);
    q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.5;
    // q << 0, 0, 0.5, 0, 0, 0;o
    state->set_manip(q, LIMB_L_ARM);

    state->robot()->setPose(state->dofs());
    Isometry3d B;
    B = left_hand->getWorldTransform();
    cout << "B = \n" << B.matrix() << endl;

    state->dofs().setZero();
    
    robot->manip_jacobian_ik(B, desired_dofs, left_hand, *state);

    state->print_joints(desired_dofs);
    
    cout << "ans = \n" << left_hand->getWorldTransform() << endl;
    
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
