#include "MyWindow.h"

#include <dynamics/SkeletonDynamics.h>
#include <kinematics/FileInfoSkel.hpp>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <robotics/World.h>

#include "utils/Paths.h"

#include <iostream>

using namespace std;
using namespace kinematics;
using namespace dynamics;
using namespace robotics;

int main(int argc, char* argv[]) {
	DartLoader dart_loader;
	World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH"atlas/atlas_world.urdf");

	SkeletonDynamics *atlas = mWorld->getSkeleton("atlas");

	mWorld->printInfo();

	FileInfoSkel<SkeletonDynamics> model;
	model.loadFile(ATLAS_DATA_PATH"/skel/ground1.skel", SKEL);

	SkeletonDynamics *ground = dynamic_cast<SkeletonDynamics *>(model.getSkel());
	ground->setName("ground");

	MyWindow window(ground, atlas);

	glutInit(&argc, argv);
	window.initWindow(640, 480, "Atlas");
    glutMainLoop();
}