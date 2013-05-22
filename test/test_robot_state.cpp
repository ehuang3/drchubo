#include <iostream>
#include <gtest/gtest.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Transformation.h>
#include <dynamics/SkeletonDynamics.h>

#include <utils/data_paths.h>
#include <robot/robot_state.h>
#include <atlas/atlas_state.h>

using namespace std;
using namespace Eigen;

using namespace kinematics;
using namespace simulation;
using namespace dynamics;

using namespace robot;
using namespace atlas;
/* ********************************************************************************************* */
#define ROBOT_URDF "models/atlas/atlas_world.urdf"
#define ROBOT_NAME "atlas"
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
TEST(STATE, TEST_INIT) {
    atlas_state_t as;
    as.init(PREPARE_ROBOT());
    atlas_state_t::print_mappings();
}
/* ********************************************************************************************* */
TEST(STATE, TEST_D_BODY) {
    atlas_state_t as;
    as.init(PREPARE_ROBOT());
    Skeleton *robot = PREPARE_ROBOT();

    Isometry3d Twb;
    Twb = Matrix4d::Identity();

    for(int i=3; i < 6; i++) {
        // cout << robot->getDof(i)->getName() << endl;
    }
    
    VectorXd dofs = robot->getPose();
    dofs.setZero();

    dofs(3) = 1;
    dofs(4) = M_PI/2;
    dofs(5) = 2;

    robot->setPose(dofs);

    Twb = robot->getNode("pelvis")->getWorldTransform();

    BodyNode *pelvis = robot->getNode("pelvis");
    Joint* joint = pelvis->getParentJoint();
    for(int i=0; i < joint->getNumTransforms(); i++) {
        Transformation *xform = joint->getTransform(i);
        cout << xform->getName() << " = \n" << xform->getTransform() << endl;
    }

    cout << endl;

    cout << "Twb = \n" << Twb.matrix() << endl;

    as.set_d_body(Twb);

    cout << "pose = \n" << as.d_pose().block<6,1>(0,0) << endl;

    robot->setPose(as.d_pose());
    
    cout << "Twb = \n" << robot->getNode("pelvis")->getWorldTransform() << endl;
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
