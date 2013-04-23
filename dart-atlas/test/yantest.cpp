#include <iostream>
#include <Eigen/Dense>
//#include <gtest/gtest.h>

#include <atlas/AtlasKinematics.h>
#include <atlas/AtlasUtils.h>
#include <utils/AtlasPaths.h>


#include <robotics/parser/dart_parser/DartLoader.h>
#include <robotics/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <dynamics/SkeletonDynamics.h>

#include <fstream>

using namespace std;
using namespace Eigen;
using namespace atlas;

using namespace kinematics;
using namespace dynamics;
using namespace robotics;


atlas::AtlasKinematics *_ak;
kinematics::Skeleton *_atlas;

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

int main()
{

	ofstream myfile;
	myfile.open("my.yaml");
	myfile << "five_steps:\n";

	int l[6], r[6];
	l[0] = 7;  //= l_leg_uhz
	l[1] = 10; //= l_leg_mhx
	l[2] = 13; //= l_leg_lhy
	l[3] = 18; //= l_leg_kny
	l[4] = 23; //= l_leg_uay
	l[5] = 27; //= l_leg_lax

	r[0] = 8;  //= r_leg_uhz
	r[1] = 11; //= r_leg_mhx
	r[2] = 14; //= r_leg_lhy
	r[3] = 19; //= r_leg_kny
	r[4] = 24; //= r_leg_uay
	r[5] = 28; //= r_leg_lax

	AtlasKinematics *AK = prepareAtlasKinematics();

	int nDofsNum = _atlas->getNumDofs();

	for(int i=0; i < nDofsNum; ++i) {
			cout << "[] = " << i << "; \\\\= " << _atlas->getDof(i)->getName() << endl;
	}

	Matrix4d Twb = _atlas->getNode("pelvis")->getWorldTransform();
	Matrix4d lf = Twb * _atlas->getNode("l_foot")->getWorldTransform();
	Matrix4d rf = Twb * _atlas->getNode("r_foot")->getWorldTransform();

//	cout << "com=\n" << _atlas->getWorldCOM() << endl;
//	cout << "Twb=\n" << Twb << endl;
//	cout << "lf=\n" << lf << endl;
//	cout << "rf=\n" << rf << endl;

	Matrix4d Twl = AK->legFK(Vector6d::Zero(), true);
	Matrix4d Twr = AK->legFK(Vector6d::Zero(), false);

//	cout << "Twl=\n" << Twl << endl;
//	cout << "Twr=\n" << Twr << endl;

//	Vector6d curr_lleg_angle, curr_lright_angle;

	/*
	for (int i = 0; i < 6; i++) {
		curr_lright_angle(i) = dofs(r[i]);
		curr_lleg_angle(i) = dofs(l[i]);
	}
	cout << "curr right leg angles:\n" << curr_lright_angle << endl;
	cout << "curr left leg angles:\n" << curr_lleg_angle << endl;
	*/

	// first 
	VectorXd dofs = _atlas->getPose();
	myfile << "  - [2, \"";
	myfile << "0 0 0 0 ";
	for (int i = 0; i < 6; i++) {
		myfile << dofs(l[i]) << ' ';
	}
	for (int i = 0; i < 6; i++) {
		myfile << dofs(r[i]) << ' ';
	}
	for (int i = 0; i < 12; i++) {
		myfile << "0 ";
	}
	myfile << "\"]\n";

	Matrix4d Tm[NUM_MANIPULATORS];
	Tm[MANIP_L_FOOT] = Twl;
	Tm[MANIP_R_FOOT] = Twr;

	IK_Mode mode[NUM_MANIPULATORS];
	mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
	mode[MANIP_R_FOOT] = IK_MODE_WORLD;
	mode[MANIP_L_HAND] = IK_MODE_FIXED;
	mode[MANIP_R_HAND] = IK_MODE_FIXED;


	Vector3d com;
	com << 0, 0, 0; // origin in world frame
	cout << "MANIP_L_FOOT:\n" << Tm[MANIP_L_FOOT] << "\n";
	cout << "MANIP_R_FOOT:\n" << Tm[MANIP_R_FOOT] << "\n";

	AK->comIK(_atlas, com, Twb, mode, Tm, dofs);


	cout << "After: \n";
//	cout << "new com relative to Twb=\n" << _atlas->getWorldCOM() << endl;;
	cout << "Twb=\n" << Twb << endl;
//	cout << "Pelvis: \n" << _atlas->getNode("pelvis")->getWorldTransform() << endl;
	cout << "MANIP_L_FOOT:\n" << Tm[MANIP_L_FOOT] << "\n";
	cout << "MANIP_R_FOOT:\n" << Tm[MANIP_R_FOOT] << "\n";

	lf = Twb * _atlas->getNode("l_foot")->getWorldTransform();
	rf = Twb * _atlas->getNode("r_foot")->getWorldTransform();
	cout << "lf=\n" << lf << endl;
	cout << "rf=\n" << rf << endl;

	Matrix<double, 4, 1> comt(4);
	comt.head(3) = _atlas->getWorldCOM();
	comt(3) = 1;

//	cout << "comt: " << comt << endl;
	cout << "new com in world frame=\n" << Twb*comt << endl;


	dofs = _atlas->getPose();
	myfile << "  - [3, \"";
	myfile << "0 0 0 0 ";
	for (int i = 0; i < 6; i++) {
		myfile << dofs(l[i]) << ' ';
	}
	for (int i = 0; i < 6; i++) {
		myfile << dofs(r[i]) << ' ';
	}
	for (int i = 0; i < 12; i++) {
		myfile << "0 ";
	}
	myfile << "\"]\n";
	myfile.close();

}
