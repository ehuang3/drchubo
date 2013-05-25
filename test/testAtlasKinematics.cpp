#include <iostream>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <atlas/atlas_kinematics.h>
#include <utils/math_utils.h>
#include <utils/data_paths.h>
#include <math/EigenHelper.h>

#include <math.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <dynamics/SkeletonDynamics.h>

#include "test_utils.h"

using namespace std;
using namespace Eigen;
using namespace atlas;
using namespace robot;

using namespace kinematics;
using namespace dynamics;
using namespace simulation;

atlas::atlas_kinematics_t *_ak;
kinematics::Skeleton *_atlas;
/* ********************************************************************************************* */
atlas::atlas_kinematics_t *prepareAtlasKinematics() {
	if(!_ak) {
		DartLoader dart_loader;
		World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH "models/atlas/atlas_world.urdf");
		_atlas = mWorld->getSkeleton("atlas");
		_ak = new atlas_kinematics_t();
		_ak->init(_atlas);
	}
	_atlas->setPose(_atlas->getPose().setZero(), true);
	return _ak;
}
/* ********************************************************************************************* */
TEST(KINEMATICS, INIT) {
	atlas_kinematics_t *AK = prepareAtlasKinematics();
}
/* ********************************************************************************************* */
TEST(KINEMATICS, FORWARD) {
	atlas_kinematics_t *AK = prepareAtlasKinematics();
	Vector6d u = Vector6d::Zero();
	Matrix4d Tfoot = AK->legFK(u, true);
}
/* ********************************************************************************************* */
TEST(KINEMATICS, COMPARE_DART_FORWARD) {
	atlas_kinematics_t *AK = prepareAtlasKinematics();

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
	//u << -0.32, 0.495, -1.75, 2.45, -0.698, 0.436;
	u << 0,0,0,0,0,0;

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
TEST(KINEMATICS, INVERSE_SINGULARITY) {
}
/* ********************************************************************************************* */
TEST(KINEMATICS, COMPARE_INVERSE_FORWARD) {
	atlas_kinematics_t *AK = prepareAtlasKinematics();

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
					ASSERT_NEAR(Tsol(r,c), Tfoot(r,c), TOLERANCE_EXACT);
		}
	}
}
/* ********************************************************************************************* */
TEST(KINEMATICS, INVERSE_NEAREST) {
	atlas_kinematics_t *AK = prepareAtlasKinematics();

	Vector6d u = Vector6d::Zero();
	Matrix4d Tfoot = AK->legFK(u, true);
	Tfoot(2,3) += 0.01;
	//cout << "Tfoot=\n" << Tfoot << endl;

	MatrixXd U;
	//AK->legIK(Tfoot, true, U);
	//cout << "U=\n" << U << endl;

	Vector6d a;
	AK->legIK(Tfoot, true, u, a);
	//cout << "ans=\n" << a << endl;

	//cout << "FK= \n" << AK->legFK(a, true) << endl;
}
/* ********************************************************************************************* */
TEST(KINEMATICS, STANCE_IK) {
	atlas_kinematics_t *AK = prepareAtlasKinematics();

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
	atlas_kinematics_t *AK = prepareAtlasKinematics();

	//cout << "com=\n" << _atlas->getWorldCOM() << endl;
	for(int i=0; i < _atlas->getNumDofs(); ++i) {
		//cout << "[] = " << i << "; \\\\= " << _atlas->getDof(i)->getName() << endl;
	}

	Matrix4d Twb = _atlas->getNode("pelvis")->getWorldTransform();
	//cout << "Twb=\n" << Twb << endl;
	Matrix4d Twl = AK->legFK(Vector6d::Zero(), true);
	Matrix4d Twr = AK->legFK(Vector6d::Zero(), false);

	Matrix4d Tm[robot::NUM_MANIPULATORS];
	Tm[robot::MANIP_L_FOOT] = Twl;
	Tm[robot::MANIP_R_FOOT] = Twr;

    robot_kinematics_t::IK_Mode mode[robot::NUM_MANIPULATORS];
	mode[robot::MANIP_L_FOOT] = robot_kinematics_t::IK_MODE_SUPPORT;
	mode[robot::MANIP_R_FOOT] = robot_kinematics_t::IK_MODE_WORLD;
	mode[robot::MANIP_L_HAND] = robot_kinematics_t::IK_MODE_FIXED;
	mode[robot::MANIP_R_HAND] = robot_kinematics_t::IK_MODE_FIXED;

	VectorXd dofs = _atlas->getPose();
	dofs.setZero();
	_atlas->setPose(dofs, true);

	Vector3d com = _atlas->getWorldCOM();

	cout << "current com=\n" << _atlas->getWorldCOM().transpose() << endl;

	com(2) -= .15;

	cout << "desired com=\n" << com.transpose() << endl;

	AK->comIK(_atlas, com, Twb, mode, Tm, dofs);

	Vector4d ncom = Vector4d::Ones();
	ncom.block<3,1>(0,0) = _atlas->getWorldCOM();

	//ncom = (Twb * ncom);

	cout << "new com=\n" << ncom.block<3,1>(0,0).transpose() << endl;

	_atlas->setPose(dofs, true);

	cout << "dofd com=\n" << _atlas->getWorldCOM().transpose() << endl;

	cout << (com - ncom.block<3,1>(0,0)).norm() << endl;


	/// Test Dart Skeleton
	dofs.setZero();
	_atlas->setPose(dofs, true);
	cout << "zero com=\n" << _atlas->getWorldCOM().transpose() << endl;

	dofs.block(0,0,3,1) = Vector3d(0, 0, 0.15);
	_atlas->setPose(dofs, true);
	cout << "0.15 com=\n" << _atlas->getWorldCOM().transpose() << endl;

	dofs.setZero();
	_atlas->setPose(dofs, true);
	cout << "pelvis=\n" << _atlas->getNode("pelvis")->getWorldTransform() << endl;

	dofs.block(0,0,3,1) = Vector3d(0, 0, 0.15);
	_atlas->setPose(dofs, true);
	cout << "pelvis=\n" << _atlas->getNode("pelvis")->getWorldTransform() << endl;

	// Assert that all joints have moved up by 0.15 m
	int nNodes = _atlas->getNumNodes();
	dofs.setZero();
	_atlas->setPose(dofs, true);
	EIGEN_V_MAT4D x0;
	for(int i=0; i < nNodes; i++) {
		x0.push_back(_atlas->getNode(i)->getWorldTransform());
	}

	dofs.block(0,0,3,1) = Vector3d(0, 0, 0.15);
	_atlas->setPose(dofs, true);
	EIGEN_V_MAT4D x1;
	for(int i=0; i < nNodes; i++) {
		x1.push_back(_atlas->getNode(i)->getWorldTransform());
	}

	double XFORM_TOL = 1e-10;
	for(int i=0; i < nNodes; i++) {

		//cout << _atlas->getNode(i)->getName() << endl;

		ASSERT_NEAR(x0[i](2,3)+0.15, x1[i](2,3), XFORM_TOL);

		for(int r=0; r < 4; r++)
		for(int c=0; c < 4; c++)
		if(!(r==2 && c==3))
		ASSERT_NEAR(x0[i](r,c), x1[i](r,c), XFORM_TOL);
	}

	// Assert that all the com contributions have moved up by 0.15
	dofs.setZero();
	_atlas->setPose(dofs, true);
	EIGEN_V_VEC3D c0;
	for(int i=0; i < nNodes; i++) {
		BodyNode *node = _atlas->getNode(i);
		c0.push_back(node->getWorldCOM());
	}

	dofs.block(0,0,3,1) = Vector3d(0, 0, 0.15);
	_atlas->setPose(dofs, true);
	EIGEN_V_VEC3D c1;
	for(int i=0; i < nNodes; i++) {
		BodyNode *node = _atlas->getNode(i);
		c1.push_back(node->getWorldCOM());
	}

	double mass = 0;
	for(int i=0; i < nNodes; i++) {
		BodyNode *node = _atlas->getNode(i);
//		cout << node->getName() << endl;
//
//		cout << "c0=\n"<< c0[i].transpose() << endl;
//		cout << "c1=\n"<< c1[i].transpose() << endl;
//
//		cout << "c0*m=\n" << (node->getMass() * c0[i]).transpose() << endl;
//		cout << "c1*m=\n" << (node->getMass() * c1[i]).transpose() << endl;

		//ASSERT_NEAR(c0[i](2), c1[i](2), XFORM_TOL);

		mass += node->getMass();
	}

	Vector3d calc_com(0,0,0);
	for(int i=0; i < nNodes; i++) {
		BodyNode *node = _atlas->getNode(i);
		calc_com += (node->getMass() * node->getWorldCOM());
	}
	calc_com = calc_com / _atlas->getMass();
	cout << "calc_com=\n" << calc_com.transpose() << endl;

	cout << "mass = " << mass << endl;
	cout << "mass = " << _atlas->getMass() << endl;

}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
