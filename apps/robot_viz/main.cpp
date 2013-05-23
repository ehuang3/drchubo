#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <kinematics/FileInfoSkel.hpp>
#include <utils/data_paths.h>

#include "MyWindow.h"

#include <iostream>
#include <string>


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
