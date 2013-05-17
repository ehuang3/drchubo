#include <dynamics/SkeletonDynamics.h>
#include <kinematics/FileInfoSkel.hpp>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

#include <utils/AtlasPaths.h>
#include <atlas/AtlasKinematics.h>
#include <zmp/ZmpGUI.h>

#include <iostream>

using namespace std;
using namespace kinematics;
using namespace dynamics;
using namespace simulation;
using namespace Eigen;

int main(int argc, char* argv[]) {
	DartLoader dart_loader;
	World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH"atlas/atlas_world.urdf");
	SkeletonDynamics *atlas = mWorld->getSkeleton("atlas");


	FileInfoSkel<SkeletonDynamics> model;
	model.loadFile(ATLAS_DATA_PATH"/skel/ground1.skel", SKEL);
	SkeletonDynamics *ground = dynamic_cast<SkeletonDynamics *>(model.getSkel());
	ground->setName("ground");


	vector<SkeletonDynamics *> skels;
	skels.push_back(ground);
	skels.push_back(atlas);


	ZmpGUI window(skels);

	// move com around
	atlas::atlas_kinematics_t AK;
	AK.init(atlas);

	Matrix4d Twb = atlas->getNode("pelvis")->getWorldTransform();
	//cout << "Twb=\n" << Twb << endl;
	Matrix4d Twl = AK.legFK(Vector6d::Zero(), true);
	Matrix4d Twr = AK.legFK(Vector6d::Zero(), false);

	Matrix4d Tm[atlas::NUM_MANIPULATORS];
	Tm[atlas::MANIP_L_FOOT] = Twl;
	Tm[atlas::MANIP_R_FOOT] = Twr;

	atlas::IK_Mode mode[atlas::NUM_MANIPULATORS];
	mode[atlas::MANIP_L_FOOT] = atlas::IK_MODE_SUPPORT;
	mode[atlas::MANIP_R_FOOT] = atlas::IK_MODE_WORLD;
	mode[atlas::MANIP_L_HAND] = atlas::IK_MODE_FIXED;
	mode[atlas::MANIP_R_HAND] = atlas::IK_MODE_FIXED;

	VectorXd dofs = atlas->getPose();

	Vector3d com;
	//com << 0, 0, 0; // origin in world frame
	com = atlas->getWorldCOM();
	com(2) = -.20;
	AK.comIK(atlas, com, Twb, mode, Tm, dofs);

	// move it down and up!
	double com_base_z = com(2);
	double com_base_y = com(1);
	double com_base_x = com(0);
	double down = -0.20;
	double shake = 0.40;
	double front = -0.40;

	// sin wave
	const int NUM_POINTS = 2000;
	double wave[NUM_POINTS];
	double coswave[NUM_POINTS];
	for(int i=0; i < NUM_POINTS; ++i) {
		wave[i] = sin(2 * M_PI * i / NUM_POINTS);
		coswave[i] = cos(2 * M_PI * i / NUM_POINTS);
	}

	// move it !!! & bake
	for(int i=0; i < NUM_POINTS; ++i) {
		com(2) = com_base_z + down * coswave[i];
		com(1) = com_base_y + shake * wave[i];
		com(0) = com_base_x + front * wave[i];
		bool ok = AK.comIK(atlas, com, Twb, mode, Tm, dofs);

		window.bake(dofs);

		if(!ok) {
			cout << "dz= " << down * wave[i] << endl;
			//break;
		}
	}

	glutInit(&argc, argv);
	window.initWindow(640, 480, "Atlas ZMP");
    glutMainLoop();
}
