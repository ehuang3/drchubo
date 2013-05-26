#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <kinematics/FileInfoSkel.hpp>
#include <utils/data_paths.h>

#include "MyWindow.h"

#include <iostream>
#include <string>

#include <robot/robot_state.h>
#include <atlas/atlas_state.h>
#include <atlas/atlas_kinematics.h>
#include <utils/robot_configs.h>


using namespace Eigen;
using namespace std;
using namespace kinematics;
using namespace dynamics;
using namespace simulation;
using namespace robot;
using namespace atlas;

int main(int argc, char* argv[])
{
    ////////////////////////////////////////////////////////////////////////////
    /// SETUP WORLD
    ////////////////////////////////////////////////////////////////////////////


	// load a robot
	DartLoader dart_loader;
	string robot_file = VRC_DATA_PATH;
	robot_file = argv[1];
	World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH ROBOT_URDF);
	SkeletonDynamics *robot = mWorld->getSkeleton("atlas");
    
    atlas_state_t state;
    state.init(robot);
    atlas_kinematics_t kin;
    kin.init(robot);

    robot->setPose(robot->getPose().setZero());

    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(VRC_DATA_PATH"/models/skel/ground1.skel", SKEL);
	SkeletonDynamics *ground = dynamic_cast<SkeletonDynamics *>(model.getSkel());
	ground->setName("ground");
	mWorld->addSkeleton(ground);
    
    // Zero atlas's feet at ground
    kinematics::BodyNode* foot = robot->getNode(ROBOT_LEFT_FOOT);
    kinematics::Shape* shoe = foot->getCollisionShape();
    double body_z = -shoe->getTransform().matrix()(2,3) + shoe->getDim()(2)/2;
    body_z += -foot->getWorldTransform()(2,3);
    VectorXd robot_dofs = robot->getPose();
    cout << "body_z = " << body_z << endl;
    state.dofs()(2) = body_z;
    state.copy_into_robot();

    cout << robot->getNode(ROBOT_BODY)->getWorldTransform() << endl;

    // Zero ground
    double ground_z = ground->getNode("ground1_root")->getCollisionShape()->getDim()(2)/2;
    ground->getNode("ground1_root")->getVisualizationShape()->setColor(Vector3d(0.5,0.5,0.5));
    VectorXd ground_dofs = ground->getPose();
    ground_dofs(1) = foot->getWorldTransform()(1,3);
    ground_dofs(2) = -ground_z;
    cout << "ground z = " << ground_z << endl;
    ground->setPose(ground_dofs);

    ////////////////////////////////////////////////////////////////////////////
    /// COM IK
    ////////////////////////////////////////////////////////////////////////////

    Vector3d world_com;
    Isometry3d end_effectors[NUM_MANIPULATORS];
    IK_Mode mode[NUM_MANIPULATORS];

    BodyNode* left_foot = state.robot()->getNode(ROBOT_LEFT_FOOT);
    BodyNode* right_foot = state.robot()->getNode(ROBOT_RIGHT_FOOT);

    end_effectors[MANIP_L_FOOT] = state.robot()->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
    end_effectors[MANIP_R_FOOT] = state.robot()->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();

    mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
    mode[MANIP_R_FOOT] = IK_MODE_WORLD;
    mode[MANIP_L_HAND] = IK_MODE_FIXED;
    mode[MANIP_R_HAND] = IK_MODE_FIXED;

    world_com = state.robot()->getWorldCOM();
    world_com(2) -= 0.1;

    Isometry3d body;
    state.get_body(body);
    body.rotate(AngleAxisd(M_PI/4, Vector3d::UnitY()));
    state.set_body(body);

    kin.com_ik(world_com, end_effectors, mode, state);

    state.copy_into_robot();

	MyWindow window;
	window.setWorld(mWorld);

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Robot Viz");
    glutMainLoop();

}
