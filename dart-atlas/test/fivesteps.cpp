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
	myfile << "five_steps:\n";
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
	Matrix4d TwlStart = _atlas->getNode("l_foot")->getWorldTransform();
	Matrix4d TwrStart = _atlas->getNode("r_foot")->getWorldTransform();

	Matrix4d Tm[NUM_MANIPULATORS];
	Tm[MANIP_L_FOOT] = TwlStart;
	Tm[MANIP_R_FOOT] = TwrStart;

	IK_Mode mode[NUM_MANIPULATORS];
	mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
	mode[MANIP_R_FOOT] = IK_MODE_WORLD;
	mode[MANIP_L_HAND] = IK_MODE_FIXED;
	mode[MANIP_R_HAND] = IK_MODE_FIXED;

//	Matrix4d Tbl = AK->legFK(Vector6d::Zero(), true);
//	Matrix4d Tbr = AK->legFK(Vector6d::Zero(), false);
//	Vector6d lleg_angle, rleg_angle;

//	double dComStartx, dComStarty, dLeftxStart, dLeftzStart, dRightxStart, dRightzStart;

	Vector6d lleg_angle, rleg_angle;
	for (int i = 0; i < 6; i++) {
		rleg_angle(i) = dofs(r[i]);
		lleg_angle(i) = dofs(l[i]);
	}
	
	/**********************
	 * First joint command
	 **********************/
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
	cout << "****************************************FIRST:" << endl;

	cout << "left foot: \n" << Tm[MANIP_L_FOOT] << endl;
	cout << "right foot: \n" << Tm[MANIP_R_FOOT] << endl;

//	dComStartx = (_atlas->getWorldCOM())(0);
//	dComStarty = (_atlas->getWorldCOM())(1);
//	dLeftxStart = Tbl(0, 3);
//	dLeftzStart = Tbl(2, 3);
//	dRightxStart = Tbr(0, 3);
//	dRightzStart = Tbr(2, 3);

//	cout << "*******************init state:" << endl;
//	cout << dComStartx << endl;
//	cout << dComStarty << endl;
//	cout << dLeftxStart << endl;
//	cout << dLeftzStart << endl;
//	cout << dRightxStart << endl;
//	cout << dRightzStart << endl;

	/***************************
	 * Second and third joint command
	 * Lower com a little amount
	 ****************************/
	com(2) -= 0.01;	



//	Tbl = AK->legFK(lleg_angle, true);
//	Tbr = AK->legFK(rleg_angle, false);

	cout << "****************************************SECOND:" << endl;
	cout << "left foot: \n" << Tm[MANIP_L_FOOT] << endl;
	cout << "right foot: \n" << Tm[MANIP_R_FOOT] << endl;
	cout << "com: \n" << com << endl;

	AK->comIK(_atlas, com, Twb, mode, Tm, dofs);
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

	Vector4d comtmp;
	comtmp.head(3) =  _atlas->getWorldCOM();
	comtmp(3) = 1;
	Vector3d comStart = (Twb * comtmp).head(3);

	cout << "comstart: " << comStart << endl;
//	/********************************
//	 * Generate sequence of joints 
//	 **********************************/
////	while ( getline(COMYFILE, sComy)) {
//	 for (int i = 0; i < 400; i++) {

//		getline(COMYFILE, sComy);
//		/*************************
//		 * Read file, get doubles
//		 ************************/
//		getline(LeftXTraFile, sLeftx);
//		getline(LeftZTraFile, sLeftz);
//		getline(RightXTraFile, sRightx);
//		getline(RightZTraFile, sRightz);

//		getline(COMXFILE, sComx);

//		dLeftx = stod(sLeftx);
//		dLeftz = stod(sLeftz);
//		dRightx = stod(sRightx);
//		dRightz = stod(sRightz);
//		dComx = stod(sComx);
//		dComy = stod(sComy);
//	
////		cout << "dComX: " << dComx << endl;

//		/*************************
//		 * comIK
//		 ************************/

//		for (int i = 0; i < 6; i++) {
//			lleg_angle(i) = dofs(l[i]);
//			rleg_angle(i) = dofs(r[i]);
//		}

//		Tbl = AK->legFK(lleg_angle, true);
//		Tbr = AK->legFK(rleg_angle, false);

//		Tm[MANIP_L_FOOT] = Twb * Tbl;
//		Tm[MANIP_L_FOOT](0, 3) = dLeftxStart + dLeftx;
//		Tm[MANIP_L_FOOT](2, 3) = dLeftzStart + dLeftz;


//		Tm[MANIP_R_FOOT] = Twb * Tbr;
//		Tm[MANIP_R_FOOT](0, 3) += dRightx;
//		Tm[MANIP_R_FOOT](2, 3) += dRightz;	

//		com(0) = dComStartx + dComx;
//		com(1) = dComStarty + dComy;

//		cout << "Target L:\n" << Tm[MANIP_L_FOOT].col(3).transpose() << endl;
//		cout << "Target R:\n" << Tm[MANIP_R_FOOT].col(3).transpose() << endl;
//		cout << "Target COM: \n" << com.transpose() << endl;

//		cout << "****************************************LOOP:" << endl;
//		cout << "left foot: \n" << Tm[MANIP_L_FOOT] << endl;
//		cout << "right foot: \n" << Tm[MANIP_R_FOOT] << endl;
//		cout << "com: \n" << com << endl;

//		cout << "i is equal to: " << i << endl;
//		AK->comIK(_atlas, com, Twb, mode, Tm, dofs);


//		/**************************
//		 * Write to file 
//		 **************************/
//		myfile << "  - [0.01, \"";
//		myfile << "0 0 0 0 ";
//		dofs = _atlas->getPose();
//		for (int i = 0; i < 6; i++) {
//			myfile << dofs(l[i]) << ' ';
//		}
//		for (int i = 0; i < 6; i++) {
//			myfile << dofs(r[i]) << ' ';
//		}
//		for (int i = 0; i < 12; i++) {
//			myfile << "0 ";
//		}
//		myfile << "\"]\n";

//		check << dLeftx << "\n";

//	}


//	/**********************
//	 * Close files
//	 **********************/
 //   LeftXTraFile.close();
 //   LeftZTraFile.close();
 //   RightXTraFile.close();
 //   RightZTraFile.close();
 //   COMXFILE.close();
 //   COMYFILE.close();
//	myfile.close();
}
