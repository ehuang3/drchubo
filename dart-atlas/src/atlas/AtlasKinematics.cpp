#include "AtlasKinematics.h"
#include "AtlasUtils.h"

#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>

#include <iostream>

using namespace Eigen;
using namespace kinematics;
using namespace std;

namespace atlas {

AtlasKinematics::AtlasKinematics() {
}

AtlasKinematics::~AtlasKinematics() {
}

void AtlasKinematics::init(Skeleton *_atlas) {
	if(_atlas) {
		for(int i=0; i < _atlas->getNumDofs(); ++i) {
			Dof *dof = _atlas->getDof(i);
			//cout << dof->getName() << "= " << dof->getValue() << endl;
		}

		// zero atlas
		VectorXd dofs = _atlas->getPose();
		VectorXd zerod_dofs(_atlas->getNumDofs());
		zerod_dofs.setZero();
		_atlas->setPose(zerod_dofs, true, false);

		// joint 1 - hip yaw
		BodyNode *LHY = _atlas->getNode("l_uglut");
		// joint 2 - hip roll
		BodyNode *LHR = _atlas->getNode("l_lglut");
		// joint 3 - hip pitch
		BodyNode *LHP = _atlas->getNode("l_uleg");
		// joint 4 - knee pitch
		BodyNode *LKP = _atlas->getNode("l_lleg");
		// joint 5 - ankle pitch
		BodyNode *LAP = _atlas->getNode("l_talus");
		// joint 6 - ankle roll
		BodyNode *LAR = _atlas->getNode("l_foot");

//		cout << "LHY\n" << LHY->getWorldTransform() << endl;
//		cout << "LHR\n" << LHR->getWorldTransform() << endl;
//		cout << "LHP\n" << LHP->getWorldTransform() << endl;
//		cout << "LKP\n" << LKP->getWorldTransform() << endl;
//		cout << "LAP\n" << LAP->getWorldTransform() << endl;
//		cout << "LAR\n" << LAR->getWorldTransform() << endl;

		// ? world origin is at Atlas's waist

		Matrix4d Tw1 = LHY->getWorldTransform();
		Matrix4d Tw3 = LHP->getWorldTransform();
		Matrix4d Tw4 = LKP->getWorldTransform();
		Matrix4d Tw5 = LAP->getWorldTransform();

		double h3;
		l1 = abs(Tw3(2,3));
		l2 = abs(Tw3(0,3));
		h3 = abs(Tw4(2,3)) - l1; // l3 is at an angle
		l4 = abs(Tw5(2,3)) - h3 - l1;

		l3 = sqrt(h3*h3 + l2*l2);

//		cout << "l1= " << l1 << endl;
//		cout << "l2= " << l2 << endl;
//		cout << "l3= " << l3 << endl;
//		cout << "l4= " << l4 << endl;

		for(int i=0; i < 6; ++i) {
			u_off[i] = 0;
		}

		// angle offs
		u_off[2] = atan2(l2, h3);
		u_off[3] = -u_off[2];

		for(int i=0; i < 6; ++i) {
			//cout << "u" << i << "= " << u_off[i] << endl;
		}

	} else {
		// dummy parameters - meaningless
		for(int i=0; i < 6; ++i) {
			u_off[i] = 0;
		}
		l1 = 1;
		l2 = 1;
		l3 = 3;
		l4 = 4;
	}

	// dh parameters
	// frame 0 - hip origin with atlas xyz orientation
	lt[0] = 0;
	ld[0] = 0;
	lr[0] = 0;
	la[0] = 0;
	// frame 1 - hip yaw
	lt[1] = M_PI/2;
	ld[1] = 0;
	lr[1] = 0;
	la[1] = M_PI/2;
	// frame 2 - hip roll
	lt[2] = -M_PI/2;
	ld[2] = l2;
	lr[2] = l1;
	la[2] = -M_PI/2;
	// frame 3 - hip pitch
	lt[3] = 0;
	ld[3] = 0;
	lr[3] = l3;
	la[3] = 0;
	// frame 4 - knee pitch
	lt[4] = 0;
	ld[4] = 0;
	lr[4] = l4;
	la[4] = 0;
	// frame 5 - ankle pitch
	lt[5] = 0;
	ld[5] = 0;
	lr[5] = 0;
	la[5] = M_PI/2;
	// frame 6 - ankle roll
	lt[6] = 0;
	ld[6] = 0;
	lr[6] = 0;
	la[6] = 0;
}

Matrix4d AtlasKinematics::legT(int _frame, double _u) {
	return _legT(_frame, _u + u_off[_frame-1]);
}

Matrix4d AtlasKinematics::legFK(const Vector6d& _u) {
	return legFK(_u(0), _u(1), _u(2), _u(3), _u(4), _u(5));
}

Matrix4d AtlasKinematics::legFK(double _u1, double _u2, double _u3, double _u4, double _u5, double _u6) {
	return _legFK(_u1 + u_off[0],
				  _u2 + u_off[1],
				  _u3 + u_off[2],
				  _u4 + u_off[3],
				  _u5 + u_off[4],
				  _u6 + u_off[5]);
}

Vector6d AtlasKinematics::legIK(const Matrix4d& _Tf, const Vector6d& _p) {
	MatrixXd u = legIK(_Tf);
	Vector6d v = _p;
	double min_dist = -1;
	for(int i=0; i < 8; ++i) {
		double dist = (u.block(0,i,6,1) - _p).norm();
		if((min_dist == -1 || dist < min_dist) && !std::isnan(dist)) {
			v = u.block(0,i,6,1);
			min_dist = dist;
		}
	}
	return v;
}

MatrixXd AtlasKinematics::legIK(const Matrix4d& _Tf) {
	MatrixXd u = _legIK(_Tf);
	for(int i=0; i < 8; ++i) {
		u(0,i) -= u_off[0];
		u(1,i) -= u_off[1];
		u(2,i) -= u_off[2];
		u(3,i) -= u_off[3];
		u(4,i) -= u_off[4];
		u(5,i) -= u_off[5];
	}
	return u;
}

Matrix4d AtlasKinematics::_legT(int _f, double _u) {
	return dh2transform(lr[_f-1], la[_f-1], lt[_f]+_u, ld[_f]);
}

Matrix4d AtlasKinematics::_legFK(double _u1, double _u2, double _u3, double _u4, double _u5, double _u6) {
	Matrix4d Tf, T01, T12, T23, T34, T45, T56;
	T01 = _legT(1, _u1);
	T12 = _legT(2, _u2);
	T23 = _legT(3, _u3);
	T34 = _legT(4, _u4);
	T45 = _legT(5, _u5);
	T56 = _legT(6, _u6);
	Tf = T01*T12*T23*T34*T45*T56;
	return Tf;
}

MatrixXd AtlasKinematics::_legIK(const Matrix4d& _Tf) {
	// % C++ implementation of script/AtlasLegIK.m

	// for convenience
	double x0,x1,x2,y0,y1,y2,z0,z1,z2,t0,t1,t2;
	x0 = _Tf(0,0); y0 = _Tf(0,1); z0 = _Tf(0,2); t0 = _Tf(0,3);
	x1 = _Tf(1,0); y1 = _Tf(1,1); z1 = _Tf(1,2); t1 = _Tf(1,3);
	x2 = _Tf(2,0); y2 = _Tf(2,1); z2 = _Tf(2,2); t2 = _Tf(2,3);

	// solve u6, u2, u1
	// % For solution method, see script/AtlasLegSymbolicKinematics.m

	// index mapping
	int m6[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };  // map of u6
	int m2[8] = { 0, 1, 2, 3, 0, 1, 2, 3 };  // map of u2
	int m1[8] = { 0, 1, 2, 3, 0, 1, 2, 3 };  // map of u1

	// sign flips for permutations
	int s2[4] = { 1, 1, -1, -1 };  // cos signs of u2
	int s4[8] = { 1, 1, 1, 1, -1, -1, -1, -1 };  // cos signs of u4

	// u6 - ankle roll
	double C1, C2;
	C1 = t0*y0 + t1*y1 + t2*y2;
	C2 = t0*x0 + t1*x1 + t2*x2;

	double u6[2];
	u6[0] = atan(-C1/C2);
	u6[1] = clamp2pi(u6[0] + M_PI);

	// u2 - hip roll
	double u2[4];
	for(int i=0; i < 4; ++i) {
		int i6 = m6[i];
		u2[i] = s2[i] * acos(y2*cos(u6[i6]) + x2*sin(u6[i6]));
	}

	// u1 - hip yaw
	double u1[4];
	for(int i=0; i < 4; ++i) {
		int i6 = m6[i];
		u1[i] = atan2( -( y1*cos(u6[i6]) + x1*sin(u6[i6]) ) / sin(u2[i]),
			           -( y0*cos(u6[i6]) + x0*sin(u6[i6]) ) / sin(u2[i]) );
	}

	// zero angles
	for(int i=0; i < 4; i++) {
		u2[i] = clamp2pi(u2[i] + M_PI/2);
		u1[i] = clamp2pi(u1[i] - M_PI/2);
	}

	// solve u3, u4, u5
	// % Geometric solution in chap 4.4 from "Introduction to Robotics" by Craig

	// u4 - knee pitch
	// u3 - hip pitch
	// u5 - ankle pitch
	double u4[8];
	double u3[8];
	double u5[8];
	Matrix4d T01, T12, T2f;
	Vector4d p2f;
	double x, y, phi, beta, psi;
	for(int i=0; i < 8; i++) {
		int i1 = m1[i];
		int i2 = m2[i];

		T01 = _legT(1, u1[i1]);
		T12 = _legT(2, u2[i2]);
		T2f = T12.inverse() * T01.inverse() * _Tf;
		p2f = T2f.block(0,3,4,1);

		x = p2f(0) - l1;
		y = -p2f(2);

		u4[i] = s4[i] * acos( (x*x + y*y - l3*l3 - l4*l4) / (2*l3*l4) );

		phi = atan2( T2f(0,2), T2f(2,2) );
		beta = atan2(y,x);
		psi = acos( (x*x + y*y + l3*l3 - l4*l4) / (2*l3*sqrt(x*x + y*y)) );

		psi = clamp2pi(psi);
		if(psi < 0)
			psi += M_PI;

		if(u4[i] < 0)
			u3[i] = beta + psi;
		else
			u3[i] = beta - psi;

		u5[i] = phi - u4[i] - u3[i];

		u3[i] = clamp2pi(u3[i]);
		u5[i] = clamp2pi(u5[i]);
	}

	// return u
	MatrixXd u(6,8);
	for(int i=0; i < 8; i++) {
		int i1 = m1[i];
		int i2 = m2[i];
		int i6 = m6[i];

		u(0,i) = u1[i1];
		u(1,i) = u2[i2];
		u(2,i) = u3[i];
		u(3,i) = u4[i];
		u(4,i) = u5[i];
		u(5,i) = u6[i6];
	}

	return u;
}

}
