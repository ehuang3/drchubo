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
	VectorXd dofs(nDofsNum);
	dofs.setZero();

	dofs(6) = 0;
	dofs(9) = 0.0017;
	dofs(12) = 0; 
	dofs(16) = 0.781;

	dofs(7) = -0.0021;
	dofs(10) = 0.0623;
	dofs(13) = -0.2654;
	dofs(18) = 0.4837; 
	dofs(23) = -0.2012; 
	dofs(27) = -0.0623;

	dofs(8) = 0.0021;
	dofs(11) = -0.0623; 
	dofs(14) = -0.2654; 
	dofs(19) = 0.4835;
	dofs(24) = -0.20122;
	dofs(28) = 0.0623,

	dofs(15) = 0.2978;
	dofs(20) = -1.3140;
	dofs(25) = 2.0021;
	dofs(29) = 0.4955;
	dofs(31) = 0; 
	dofs(33) = -0.01;

	dofs(17) = 0.2978;
	dofs(22) = 1.3140;
	dofs(26) = 2.0021;
	dofs(30) = -0.4955; 
	dofs(32) = 0;
	dofs(34) = 0.01;

	cout << "before set: " << dofs << endl;
	_atlas->setPose(dofs, true, false);
	dofs = _atlas->getPose();
	cout << "after set: " << dofs << endl;


	cout << "left foot: \n" << _atlas->getNode("l_foot")->getWorldTransform() << endl;
	cout << "right foot: \n" << _atlas->getNode("r_foot")->getWorldTransform() << endl;

	Vector3d com, comStart;
	comStart = com = _atlas->getWorldCOM();
	cout << "com: " << comStart << endl;

	Matrix4d Twb;
	Twb.setIdentity();


	Vector6d lleg_angle, rleg_angle;
	for (int i = 0; i < 6; i++) {
		rleg_angle(i) = dofs(r[i]);
		lleg_angle(i) = dofs(l[i]);
	}

	Matrix4d TwlStart = AK->legFK(lleg_angle, true);
//	TwlStart.col(3) = _atlas->getNode("l_foot")->getWorldTransform().col(3);
	Matrix4d TwrStart = AK->legFK(rleg_angle, false);
//	TwrStart.col(3) = _atlas->getNode("r_foot")->getWorldTransform().col(3);

	Matrix4d Tm[NUM_MANIPULATORS];
	Tm[MANIP_L_FOOT] = TwlStart;
	Tm[MANIP_R_FOOT] = TwrStart;

	IK_Mode mode[NUM_MANIPULATORS];
	mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
	mode[MANIP_R_FOOT] = IK_MODE_WORLD;
	mode[MANIP_L_HAND] = IK_MODE_FIXED;
	mode[MANIP_R_HAND] = IK_MODE_FIXED;

	cout << "com: " << comStart << endl;
	cout << "TwlStart: " << TwlStart << endl;
	cout << "TwrStart: " << TwrStart << endl;
	
	/********************************
	 * Generate sequence of joints 
	 **********************************/

	while ( getline(COMYFILE, sComy)) {

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
	

		/*************************
		 * comIK
		 ************************/


		cout << "currentCom: \n" << com.transpose() << endl;
		cout << "currentLeft: \n" << Tm[MANIP_L_FOOT] << endl;
		cout << "currentRight: \n" << Tm[MANIP_R_FOOT] << endl;
		cout << "currentTwb: \n" << Twb << endl;

		Tm[MANIP_L_FOOT] = TwlStart;
		Tm[MANIP_L_FOOT](0, 3) += dLeftx;
		Tm[MANIP_L_FOOT](2, 3) += dLeftz;

		Tm[MANIP_R_FOOT] = TwrStart;
		Tm[MANIP_R_FOOT](0, 3) += dRightx;
		Tm[MANIP_R_FOOT](2, 3) += dRightz;


		com = comStart;
		com(0) += dComx;
		com(1) += dComy;

		cout << "desiredCom: \n" << com.transpose() << endl;
		cout << "desiredLeft: \n" << Tm[MANIP_L_FOOT] << endl;
		cout << "desiredRight: \n" << Tm[MANIP_R_FOOT] << endl;

		if (AK->comIK(_atlas, com, Twb, mode, Tm, dofs) != true) {
			cout << "comIK failed!" << endl;
			exit(1);
		}
		else 
			cout << "comIK success" << endl;


		/**************************
		 * Write to file 
		 **************************/
		myfile << "  - [0.001, \"";
		myfile << dofs(6) << ' ';
		myfile << dofs(9) << ' ';
		myfile << dofs(12) << ' ';
		myfile << dofs(16) << ' ';
	
	
		dofs = _atlas->getPose();
		for (int i = 0; i < 6; i++) {
			myfile << dofs(l[i]) << ' ';
		}
		for (int i = 0; i < 6; i++) {
			myfile << dofs(r[i]) << ' ';
		}

		myfile << dofs(15) << ' ';
		myfile << dofs(20) << ' ';
		myfile << dofs(25) << ' ';
		myfile << dofs(29) << ' ';
		myfile << dofs(31) << ' ';
		myfile << dofs(33) << ' ';
		myfile << dofs(17) << ' ';
		myfile << dofs(22) << ' ';
		myfile << dofs(26) << ' ';
		myfile << dofs(30) << ' ';
		myfile << dofs(32) << ' ';
		myfile << dofs(34) << ' ';

			
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
