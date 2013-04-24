#include "MyWindow.h"

#include <dynamics/SkeletonDynamics.h>
#include <kinematics/FileInfoSkel.hpp>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <robotics/World.h>

#include <utils/AtlasPaths.h>
#include <atlas/AtlasKinematics.h>

#include <iostream>

using namespace std;
using namespace kinematics;
using namespace dynamics;
using namespace robotics;
using namespace Eigen;
using namespace atlas;

atlas::AtlasKinematics *_ak;
kinematics::Skeleton *_atlas;

/* ********************************************************************************************* */
atlas::AtlasKinematics *prepareAtlasKinematics() {
	if(!_ak) {
		DartLoader dart_loader;
		World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH "atlas/atlas_world.urdf");
		_atlas = mWorld->getSkeleton("atlas");
		_ak = new AtlasKinematics();
		_ak->init(_atlas);
	}
	_atlas->setPose(_atlas->getPose().setZero(), true);
	return _ak;
}

int main(int argc, char* argv[]) {

	DartLoader dart_loader;
	World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH"atlas/atlas_world.urdf");
	SkeletonDynamics *atlas = mWorld->getSkeleton("atlas");

	AtlasKinematics *AK = prepareAtlasKinematics();

	FileInfoSkel<SkeletonDynamics> model;
	model.loadFile(ATLAS_DATA_PATH"/skel/ground1.skel", SKEL);
	SkeletonDynamics *ground = dynamic_cast<SkeletonDynamics *>(model.getSkel());
	ground->setName("ground");

	vector<SkeletonDynamics *> skels;
	skels.push_back(ground);
	skels.push_back(atlas);

	MyWindow window(skels);

	vector<VectorXd> allDofs = window.dofs();

	/**************************************
	 * Init, World start from Atlas' pelvis
	 **************************************/
	Matrix4d Twb;
	Twb.setIdentity();
	assert( (_atlas->getNode("pelvis")->getWorldTransform() - Twb).norm() < 1e-7);

	Matrix4d Twm[NUM_MANIPULATORS];

	/******************************
	 * Some vars
	 ******************************/
	IK_Mode mode[NUM_MANIPULATORS];
	mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
	mode[MANIP_R_FOOT] = IK_MODE_WORLD;
	mode[MANIP_L_HAND] = IK_MODE_FIXED;
	mode[MANIP_R_HAND] = IK_MODE_FIXED;

	Vector3d dcom;
	int N = 6000;

	int nDofsNum = _atlas->getNumDofs();
	VectorXd dofs(nDofsNum);
	dofs.setZero();

	/*************************
	 * Move whole body, after that:
	 * World starts
	 *************************/


	dofs(6) = -3.852963655681663e-06;
	dofs(9) = 0.0009848090520510056;
	dofs(12) = 0.00010776096065789886; 
	dofs(16) = 0.7806726847358707;

//	dofs(7) = -0.0012852229728474995;
//	dofs(10) = 0.061783845913243596;
	dofs(13) = -0.2856717835152658;
	dofs(18) = 0.5266262672930972; 
	dofs(23) = -0.21864856475431704; 
//	dofs(27) = -0.062394234133471116;

//	dofs(8) = 0.0013642411907399676;
//	dofs(11) = -0.06195623610921519; 
	dofs(14) = -0.2865128374461472; 
	dofs(19) = 0.5268958272322948;
	dofs(24) = -0.21621680953724365;
//	dofs(28) = 0.06138342176132294;

	dofs(15) = 0.2988754829726865;
	dofs(20) = -1.3138418263023999;
	dofs(25) = 2.00219166466513;
	dofs(29) = 0.4950932275951452;
	dofs(31) = -8.449255747589035e-05; 
	dofs(33) = -0.010542899622185686;

	dofs(17) = 0.298867201336498138;
	dofs(22) = 1.313736629564075;
	dofs(26) = 2.0021752327042464;
	dofs(30) = -0.49508063541354375; 
	dofs(32) = -8.39712346438759e-05;
	dofs(34) = 0.01056157776909128;


	int _N = 3000;
	VectorXd _old = VectorXd::Zero(nDofsNum);
	VectorXd _new = dofs;
	VectorXd tmp = _old;
	VectorXd delta = (_new - _old) / _N;

	_atlas->setPose(dofs, true);

	for ( int i = 0; i < _N; i++) {

		allDofs[1] = tmp;

		window.bake(allDofs);

		tmp += delta;
	}
	
	/*************************
	 * Move COM down
	 *************************/
	N = 3000;
	double theta = M_PI / (N-1);	 

	Vector3d comStart = AK->getCOMW(_atlas, Twb);
	dcom = comStart;

	Vector3d comDelta;
	comDelta << 0, 0, -0.07;

//	delta = -0.07 / (N-1);		// move down
	dofs = _atlas->getPose();


	cout << "dcom start: " << dcom.transpose() << endl;
	cout << "left foot start:\n" << Twm[MANIP_L_FOOT] << endl; 

	comDelta /= 2;
	cout << "comDelta: " << comDelta.transpose() << endl;

	for (int i = 0; i < N; i++) {

		dcom = comStart + comDelta * (1 - cos(theta * i));
//		cout << "dcom: " << dcom.transpose() << endl;
		Twm[MANIP_L_FOOT] = AK->getLimbTransW(_atlas, Twb, MANIP_L_FOOT);
		Twm[MANIP_R_FOOT] = AK->getLimbTransW(_atlas, Twb, MANIP_R_FOOT);

		assert(AK->comIK(_atlas, dcom, Twb, mode, Twm, dofs) == true);
		dofs = _atlas->getPose();

		allDofs[1] = dofs;

		window.bake(allDofs);


	}

	cout << "dcom end: " << dcom.transpose() << endl;
	cout << "left foot end:\n" << Twm[MANIP_L_FOOT] << endl; 

	/*******************************
	 * Move COM to right root
	 *******************************/
	N = 3000;
	theta = M_PI / (N-1);	 
	double offset = 0;

	comStart = AK->getCOMW(_atlas, Twb);
	dcom = comStart;
	comDelta << 0, ( Twm[MANIP_R_FOOT](1,3) - dcom(1) + offset), 0;

	cout << "R FOOT: \n" << Twm[MANIP_R_FOOT] << endl;
	cout << "com start: " << dcom.transpose() << endl;

	comDelta /= 2;
	cout << "com delta: " << comDelta.transpose() << endl;
	dofs = _atlas->getPose();

	for (int i = 0; i < N; i++) {
		dcom = comStart + comDelta * (1 - cos(theta * i));

		Twm[MANIP_L_FOOT] = AK->getLimbTransW(_atlas, Twb, MANIP_L_FOOT);
		Twm[MANIP_R_FOOT] = AK->getLimbTransW(_atlas, Twb, MANIP_R_FOOT);

		assert(AK->comIK(_atlas, dcom, Twb, mode, Twm, dofs) == true);
		dofs = _atlas->getPose();

		allDofs[1] = dofs;

		window.bake(allDofs);

	}

	/******************************
	 * Lift up left foot
	 ********************************/
	N = 3000;
	theta = M_PI / (N-1);	 

	Matrix4d mDelta = Matrix4d::Zero();
	Matrix4d dm = mDelta;
	mDelta(2,3) = 0.05; 				// lift up
	Matrix4d mStart = Twm[MANIP_L_FOOT];


	mDelta /= 2;
	cout << "left foot start: \n" << mStart << endl;
	cout << "left foot delta: \n" << mDelta << endl;

	dofs = _atlas->getPose();

	for (int i = 0; i < N; i++) {

		Twm[MANIP_L_FOOT] = mStart + mDelta * (1 - cos(theta * i));
//		Twm[MANIP_R_FOOT] = AK->getLimbTransW(_atlas, Twb, MANIP_R_FOOT);

		assert(AK->comIK(_atlas, dcom, Twb, mode, Twm, dofs) == true);
		dofs = _atlas->getPose();

		allDofs[1] = dofs;

		window.bake(allDofs);

	}

	glutInit(&argc, argv);
	window.initWindow(640, 480, "Atlas IK Demo");
    glutMainLoop();
}
