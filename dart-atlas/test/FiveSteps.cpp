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
#include <string>

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

	/*************************
	 * Output files
	 *************************/
	ofstream myfile;
	myfile.open("my.yaml");
//	myfile << "five_steps:\n";
	ofstream check;
	check.open("check");


	/************************
	 * Input files
	 ************************/
	string sLeftx, sLeftz, sRightx, sRightz, sComx, sComy;
 	double dLeftx, dLeftz, dRightx, dRightz, dComx, dComy;
	ifstream LeftXTraFile, RightXTraFile, LeftZTraFile, RightZTraFile, COMXFILE, COMYFILE;

    LeftXTraFile.open("leftx");
    LeftZTraFile.open("leftz");
    RightXTraFile.open("rightx");
    RightZTraFile.open("rightz");
	COMXFILE.open("comx");
	COMYFILE.open("comy");


    /***************************
     * DOF number in dart
     ***************************/
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


	/******************
	 * Declare and Init
	 ******************/
	AtlasKinematics *AK = prepareAtlasKinematics();

	cout << endl;
	int nDofsNum = _atlas->getNumDofs();
	VectorXd dofs = _atlas->getPose();

	Vector3d com = _atlas->getWorldCOM();

	Matrix4d Twb = _atlas->getNode("pelvis")->getWorldTransform();
	Matrix4d TwlStart = AK->legFK(Vector6d::Zero(), true);
	Matrix4d TwrStart = AK->legFK(Vector6d::Zero(), false);

	Matrix4d Tm[NUM_MANIPULATORS];
	Tm[MANIP_L_FOOT] = TwlStart;
	Tm[MANIP_R_FOOT] = TwrStart;

	IK_Mode mode[NUM_MANIPULATORS];
	mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
	mode[MANIP_R_FOOT] = IK_MODE_WORLD;
	mode[MANIP_L_HAND] = IK_MODE_FIXED;
	mode[MANIP_R_HAND] = IK_MODE_FIXED;


	// lower
	com(2) -= 0.01;	

	AK->comIK(_atlas, com, Twb, mode, Tm, dofs);
	dofs = _atlas->getPose();

//	Vector6d lleg_angle, rleg_angle;
//	for (int i = 0; i < 6; i++) {
//		rleg_angle(i) = dofs(r[i]);
//		lleg_angle(i) = dofs(l[i]);
//	}

//	/********************************
//	 * Generate sequence of joints 
//	 **********************************/

	Vector4d comtmp;
	comtmp.head(3) =  _atlas->getWorldCOM();
	comtmp(3) = 1;
	Vector3d comStart = (Twb * comtmp).head(3);

	cout << "comstart: " << comStart << endl;
	cout << "TwlStart: " << TwlStart << endl;

//	cout << "XXXXXX: " << Twb * AK->legFK(rleg_angle, true) << endl;
	while ( getline(COMYFILE, sComy)) {
//	for (int i = 0; i < 1; i++) {

//		getline(COMYFILE, sComy);
		/*************************
		 * Read file, get doubles
		 ************************/
		getline(LeftXTraFile, sLeftx);
		getline(LeftZTraFile, sLeftz);
		getline(RightXTraFile, sRightx);
		getline(RightZTraFile, sRightz);

		getline(COMXFILE, sComx);

		dLeftx = stod(sLeftx);
		dLeftz = stod(sLeftz);
		dRightx = stod(sRightx);
		dRightz = stod(sRightz);
		dComx = stod(sComx);
		dComy = stod(sComy);
	
//		cout << "dComx: " << dComx << endl;

		/*************************
		 * comIK
		 ************************/

		Tm[MANIP_L_FOOT] = TwlStart;
		Tm[MANIP_L_FOOT](0, 3) += dLeftx;
		Tm[MANIP_L_FOOT](2, 3) += dLeftz;

		Tm[MANIP_R_FOOT] = TwrStart;
		Tm[MANIP_R_FOOT](0, 3) += dRightx;
		Tm[MANIP_R_FOOT](2, 3) += dRightz;

		com = comStart;
		com(0) += dComx;
		com(1) += dComy;

//		cout << "INCREMENTAL: " << dLeftx << dLeftz << dRightx << dRightz << dComx << dComy << endl;
//		cout << "Current R: " << Twb * (AK->legFK(rleg_angle, true)) << endl;
//		cout << "Current COM: " << comStart;

//		cout << "Target L:\n" << Tm[MANIP_L_FOOT].col(3).transpose() << endl;
//		cout << "Target R:\n" << Tm[MANIP_R_FOOT].col(3).transpose() << endl;
//		cout << "Target COM: \n" << com.transpose() << endl;

//		cout << "****************************************LOOP:" << endl;
//		cout << "left foot: \n" << Tm[MANIP_L_FOOT] << endl;
//		cout << "right foot: \n" << Tm[MANIP_R_FOOT] << endl;
//		cout << "com: \n" << com << endl;

//		cout << "i is equal to: " << i << endl;
		if (AK->comIK(_atlas, com, Twb, mode, Tm, dofs) != true) {
			cout << "comIK failed!" << endl;
			exit(1);
		}


		/**************************
		 * Write to file 
		 **************************/
		myfile << "  - [0.001, \"";
		myfile << "0 0 0 0 ";
		dofs = _atlas->getPose();
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

		check << dLeftx << "\n";

	}


	/**********************
	 * Close files
	 **********************/
   	LeftXTraFile.close();
   	LeftZTraFile.close();
   	RightXTraFile.close();
   	RightZTraFile.close();
   	COMXFILE.close();
   	COMYFILE.close();
	myfile.close();
}
