#include <iostream>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <atlas/AtlasKinematics.h>
#include <atlas/AtlasUtils.h>
#include <utils/AtlasPaths.h>

#include <math.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <robotics/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <dynamics/SkeletonDynamics.h>

using namespace std;
using namespace Eigen;
using namespace atlas;

using namespace kinematics;
using namespace dynamics;
using namespace robotics;

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
/* ********************************************************************************************* */
TEST(KINEMATICS, INIT) {
	AtlasKinematics *AK = prepareAtlasKinematics();
}
/* ********************************************************************************************* */
TEST(KINEMATICS, FORWARD) {
	AtlasKinematics *AK = prepareAtlasKinematics();
	Vector6d u = Vector6d::Zero();
	Matrix4d Tfoot = AK->legFK(u, true);
}
/* ********************************************************************************************* */
TEST(KINEMATICS, COMPARE_DART_FORWARD) {
	AtlasKinematics *AK = prepareAtlasKinematics();

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

	Vector6d u;
	u << -0.32, 0.495, -1.75, 2.45, -0.698, 0.436;

	VectorXd dofs = _atlas->getPose();
	for(int i=0; i < 6; ++i) {
		dofs(l[i]) = u(i);
		dofs(r[i]) = u(i);
	}
	_atlas->setPose(dofs, true);

	cout << "FK left=\n" << AK->legFK(u, true) << endl;
	cout << "FK right=\n" << AK->legFK(u, false) << endl;
	cout << "DART left=\n" << _atlas->getNode("l_foot")->getWorldTransform() << endl;
	cout << "DART right=\n" << _atlas->getNode("r_foot")->getWorldTransform() << endl;
}
/* ********************************************************************************************* */
TEST(KINEMATICS, INVERSE_SELECTION) {
	AtlasKinematics *AK = prepareAtlasKinematics();

	Vector6d u = Vector6d::Zero();
	Matrix4d Tf = AK->legFK(u, true);
	MatrixXd V;
	AK->legIK(Tf, true, V);

	cout << "V=\n" << V << endl;
}
/* ********************************************************************************************* */
TEST(KINEMATICS, INVERSE_SINGULARITY) {
}
/* ********************************************************************************************* */
TEST(KINEMATICS, COMPARE_INVERSE_FORWARD) {
	AtlasKinematics *AK = prepareAtlasKinematics();

	const double TOLERANCE_EXACT = 1.0e-10;
	Vector6d u(6);
	Matrix4d Tfoot, Tsol;
	MatrixXd U;

	// joint limits
	double u_lim[6][2];
	u_lim[0][0] = -0.32;
	u_lim[0][1] = 1.14;
	u_lim[1][0] = -0.47;
	u_lim[1][1] = 0.495;
	u_lim[2][0] = -1.75;
	u_lim[2][1] = 0.524;
	u_lim[3][0] = 0;
	u_lim[3][1] = 2.45;
	u_lim[4][0] = -0.698;
	u_lim[4][1] = 0.698;
	u_lim[5][0] = -0.436;
	u_lim[5][1] = 0.436;

	// generate angles
	const int NUM_SLICE = 7;
	double a[6][NUM_SLICE];
	for(int i=0; i < 6; ++i)
	for(int j=0; j < NUM_SLICE; ++j) {
		double incr = (u_lim[i][1] - u_lim[i][0])/(NUM_SLICE-1);
		a[i][j] = u_lim[i][0] + j*incr;
	}

	// test
	for(int i1=0; i1 < NUM_SLICE; ++i1)
	for(int i2=0; i2 < NUM_SLICE; ++i2)
	for(int i3=0; i3 < NUM_SLICE; ++i3)
	for(int i4=0; i4 < NUM_SLICE; ++i4)
	for(int i5=0; i5 < NUM_SLICE; ++i5)
	for(int i6=0; i6 < NUM_SLICE; ++i6) {
		u(0) = a[0][i1];
		u(1) = a[1][i2];
		u(2) = a[2][i3];
		u(3) = a[3][i4];
		u(4) = a[4][i5];
		u(5) = a[5][i6];
		Tfoot = AK->legFK(u, true);

//		cout << "u=\n" << u << endl;
//		cout << "Tfoot=\n" << Tfoot << endl;

		AK->legIK(Tfoot, true, u, u);

		for(int i=0; i < u.cols(); ++i) {
			// compare IK FK
			Tsol = AK->legFK(u.block(0,i,6,1), true);
			for(int r=0; r < 4; ++r)
				for(int c=0; c < 4; ++c)
					EXPECT_NEAR(Tsol(r,c), Tfoot(r,c), TOLERANCE_EXACT);
		}
	}
}
/* ********************************************************************************************* */
TEST(KINEMATICS, INVERSE_NEAREST) {
	AtlasKinematics *AK = prepareAtlasKinematics();

	Vector6d u = Vector6d::Zero();
	Matrix4d Tfoot = AK->legFK(u, true);
	Tfoot(2,3) += 0.01;
	//cout << "Tfoot=\n" << Tfoot << endl;

	MatrixXd U;
	AK->legIK(Tfoot, true, U);
	//cout << "U=\n" << U << endl;

	Vector6d a;
	AK->legIK(Tfoot, true, u, a);
	//cout << "ans=\n" << a << endl;

	//cout << "FK= \n" << AK->legFK(a, true) << endl;
}
/* ********************************************************************************************* */
TEST(KINEMATICS, STANCE_IK) {
	AtlasKinematics *AK = prepareAtlasKinematics();

	VectorXd u(12), v(12);
	u.setZero();

	Matrix4d Twb = Matrix4d::Identity();
	Matrix4d Twl = AK->legFK(u.topRows(6), true);
	Matrix4d Twr = AK->legFK(u.bottomRows(6), false);

	cout << "Twl=\n" << Twl << endl;
	cout << "Twr=\n" << Twr << endl;

	AK->stanceIK(Twb, Twl, Twr, u, v);

	// generate sin waves make sure atlas lifts his foot
	const double TOLERANCE_EXACT = 1.0e-10;
	double base_loc = Twl(2,3);
	double up = 0.2;
	double forward = 0.2;
	const int NUM_POINTS = 1000;
	double wave[NUM_POINTS];
	for(int i=0; i < NUM_POINTS; i++) {
		wave[i] = sin(M_PI * i / NUM_POINTS);
	}
	for(int i=0; i < NUM_POINTS; i++) {
		// z
		Twl(2,3) = base_loc + up * wave[i];
		Twr(2,3) = base_loc + up * wave[i];
		// x
		Twl(0,3) = forward * wave[i];
		Twr(0,3) = forward * wave[i];


		AK->stanceIK(Twb, Twl, Twr, u, u);

		Matrix4d Fwl = AK->legFK(u.topRows(6), true);
		Matrix4d Fwr = AK->legFK(u.bottomRows(6), false);

		//		cout << "Twl=\n" << Twl << endl;
		//		cout << "Fwl=\n" << Fwl << endl;
		//
		//		cout << "Twr=\n" << Twr << endl;
		//		cout << "Fwr=\n" << Fwr << endl;

		for(int i=0; i < 4; i++)
			for(int j=0; j < 4; j++) {
				ASSERT_NEAR(Fwl(i,j), Twl(i,j), TOLERANCE_EXACT);
				ASSERT_NEAR(Fwr(i,j), Twr(i,j), TOLERANCE_EXACT);
			}
	}
}
/* ********************************************************************************************* */
TEST(KINEMATICS, COM_IK) {
	AtlasKinematics *AK = prepareAtlasKinematics();

	cout << "com=\n" << _atlas->getWorldCOM() << endl;
	for(int i=0; i < _atlas->getNumDofs(); ++i) {
		//cout << "[] = " << i << "; \\\\= " << _atlas->getDof(i)->getName() << endl;
	}

	Matrix4d Twb = _atlas->getNode("pelvis")->getWorldTransform();
	//cout << "Twb=\n" << Twb << endl;
	Matrix4d Twl = AK->legFK(Vector6d::Zero(), true);
	Matrix4d Twr = AK->legFK(Vector6d::Zero(), false);

	Matrix4d Tm[NUM_MANIPULATORS];
	Tm[MANIP_L_FOOT] = Twl;
	Tm[MANIP_R_FOOT] = Twr;

	IK_Mode mode[NUM_MANIPULATORS];
	mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
	mode[MANIP_R_FOOT] = IK_MODE_WORLD;
	mode[MANIP_L_HAND] = IK_MODE_FIXED;
	mode[MANIP_R_HAND] = IK_MODE_FIXED;

	VectorXd dofs = _atlas->getPose();

	Vector3d com;
	com << 0, 0, 0; // origin in world frame

	AK->comIK(_atlas, com, Twb, mode, Tm, dofs);

	//cout << "new com=\n" << _atlas->getWorldCOM();
}


/* ********************************************************************************************* */
TEST(KINEMATICS, UTILS_InitPos) {

	AtlasKinematics *AK = prepareAtlasKinematics();

	/**************************************
	 * Init, World start from Atlas' pelvis
	 **************************************/
	Matrix4d Twb;
	Twb.setIdentity();
	assert( (_atlas->getNode("pelvis")->getWorldTransform() - Twb).norm() < 1e-7);

	Matrix4d Twm[NUM_MANIPULATORS];
	Twm[MANIP_L_FOOT] = AK->getLimbTransB(_atlas, MANIP_L_FOOT);
	Twm[MANIP_R_FOOT] = AK->getLimbTransB(_atlas, MANIP_R_FOOT);


	/************************
	 * Trajectory file
	 ************************/
	ofstream file;
	file.open("what.yaml");
	file << "MovCOM:\n";


	/******************************
	 * Some vars
	 ******************************/
	IK_Mode mode[NUM_MANIPULATORS];
	mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
	mode[MANIP_R_FOOT] = IK_MODE_WORLD;
	mode[MANIP_L_HAND] = IK_MODE_FIXED;
	mode[MANIP_R_HAND] = IK_MODE_FIXED;

	Vector3d dcom;
	double delta; 
	int N;

	int nDofsNum = _atlas->getNumDofs();
	VectorXd dofs(nDofsNum);
	dofs.setZero();


	/*************************
	 * Move whole body
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

	_atlas->setPose(dofs, true);

	AK->writeTrajectory(file, VectorXd::Zero(nDofsNum), dofs, 3000, false);


	/*************************
	 * Move COM down
	 *************************/
	N = 3000;
	double theta = M_PI / (N-1);	 

	Vector3d comStart = AK->getCOMW(_atlas, Twb);
	dcom = comStart;

	Vector3d comDelta;
	comDelta << 0, 0, -0.07;

//	delta = -0.07 / (N-1);		// move 15cm down
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

		file << "  - [\"";
		file << dofs(6) << ' ';
		file << dofs(9) << ' ';
		file << dofs(12) << ' ';
		file << dofs(16) << ' ';

		for (int j = 0; j < NUM_MANIPULATORS; j++) {
			for (int k = 0; k < 6; k++) {
				file << dofs(AK->dof_ind[j][k]) << ' ';
			}
		}

		file << "\"]\n";
//		dcom(2) += delta;


	}

	cout << "dcom end: " << dcom.transpose() << endl;
	cout << "left foot end:\n" << Twm[MANIP_L_FOOT] << endl; 

//	AK->printGazeboAngles(_atlas, _atlas->getPose());


	/*******************************
	 * Move COM to right root
	 *******************************/
	N = 3000;
	theta = M_PI / (N-1);	 
	double offset = 0;

	comStart = AK->getCOMW(_atlas, Twb);
	dcom = comStart;
	comDelta << 0, ( Twm[MANIP_R_FOOT](1,3) - dcom(1) + offset ), 0;

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

		file << "  - [\"";
		file << dofs(6) << ' ';
		file << dofs(9) << ' ';
		file << dofs(12) << ' ';
		file << dofs(16) << ' ';

		for (int j = 0; j < NUM_MANIPULATORS; j++) {
			for (int k = 0; k < 6; k++) {
				file << dofs(AK->dof_ind[j][k]) << ' ';
			}
		}

		file << "\"]\n";

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
		Twm[MANIP_R_FOOT] = AK->getLimbTransW(_atlas, Twb, MANIP_R_FOOT);

		assert(AK->comIK(_atlas, dcom, Twb, mode, Twm, dofs) == true);
		dofs = _atlas->getPose();

		file << "  - [\"";
		file << dofs(6) << ' ';
		file << dofs(9) << ' ';
		file << dofs(12) << ' ';
		file << dofs(16) << ' ';

		for (int j = 0; j < NUM_MANIPULATORS; j++) {
			for (int k = 0; k < 6; k++) {
				file << dofs(AK->dof_ind[j][k]) << ' ';
			}
		}

		file << "\"]\n";

	}



	/*********************************
	 * Print foot transform
	 *********************************/
	// close file
	file.close();


}


/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
