#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <kinematics/FileInfoSkel.hpp>
#include <utils/data_paths.h>

#include "MyWindow.h"

#include <iostream>
#include <string>

#include "robot/robot.h"
#include "hubo/hubo_state.h"
#include "hubo/hubo_jacobian.h"

using namespace Eigen;
using namespace std;
using namespace kinematics;
using namespace dynamics;
using namespace simulation;

int main(int argc, char* argv[])
{
	if(argc == 1) {
		cout << "usage: robot_viz <robot_world.urdf>" << endl;
		exit(1);
	}

	// load a robot
	DartLoader dart_loader;
	string robot_file = VRC_DATA_PATH;
	robot_file = argv[1];
	World *mWorld = dart_loader.parseWorld(robot_file.c_str());
	SkeletonDynamics *robot = mWorld->getSkeleton(0);

    robot->setPose(robot->getPose().setZero(), true);

    // Print out all dofs
    // for(int i=0; i < robot->getNumJoints(); ++i) {
    //     cout << "ros2s[" << i << "] = \"hubo::" << robot->getJoint(i)->getName() << "\";\n";
    // }

    // for(int i=0; i < robot->getNumNodes(); ++i) {
    //     cout << "node " << robot->getNode(i)->getName() << endl;
    // }

    // Jacobian arm IK
    hubo::hubo_state_t state;
    hubo::hubo_jacobian_t rjac;
    state.init(robot);
    rjac.init(robot);

    vector<int> desired;
    state.get_manip_indexes(desired, robot::LIMB_L_ARM);
    VectorXd q(desired.size());

    for(int i=0; i < q.rows(); i++)
        q(i) = 0.1 * i;

    state.set_manip(q, robot::LIMB_L_ARM);
    state.copy_into_robot();

    BodyNode *left_hand = state.robot()->getNode("leftPalm");

    Isometry3d B;
    B = state.robot()->getNode("leftPalm")->getWorldTransform();

    state.dofs().setZero();
    state.copy_into_robot();

    cout << "B = \n" << B.matrix() << endl;

    rjac.manip_jacobian_ik(B, desired, left_hand, state);

    state.copy_into_robot();

    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(VRC_DATA_PATH"/skel/ground1.skel", SKEL);
	SkeletonDynamics *ground = dynamic_cast<SkeletonDynamics *>(model.getSkel());
	ground->setName("ground");
	//mWorld->addSkeleton(ground);

	MyWindow window;
	window.setWorld(mWorld);

    glutInit(&argc, argv);
    window.initWindow(640, 480, "Robot Viz");
    glutMainLoop();

}
