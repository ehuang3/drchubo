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

#include <atlas/atlas_jacobians.h>

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
void DART_FK(Isometry3d& B, const VectorXd& q, int side) {

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
    atlas_jacobians_t aj;
    aj.init(PREPARE_ROBOT());
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_SINGLE_ARM) {
    Skeleton* robot = PREPARE_ROBOT();
    MatrixXd jac_lin = robot->getNode(LEFT_HAND)->getJacobianLinear();
    MatrixXd jac_ang = robot->getNode(LEFT_HAND)->getJacobianAngular();
    cout << "jac_lin = \n" << jac_lin << endl;
    cout << "jac_ang = \n" << jac_ang << endl;
    // Try damped least squares
    int m = jac_lin.rows();
    int n = jac_lin.cols();
    VectorXd q(jac_lin.cols());
    Vector3d x;
    x << 0.01, 0, 0;
    aa_la_dls(m, n, 0.1, jac_lin.data(), x.data(), q.data());
    cout << "dls q = \n" << q << endl;
    
    PRINT_KINEMATIC_CHAIN(robot->getNode(LEFT_HAND));
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
