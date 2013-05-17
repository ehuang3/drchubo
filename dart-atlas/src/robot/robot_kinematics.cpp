#include "robot_kinematics.h"
#include "utils/math_utils.h"

#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>

#include <iostream>
#include <complex>

using namespace Eigen;
using namespace kinematics;
using namespace std;

namespace robot {

robot_kinematics_t::robot_kinematics_t() {
}

robot_kinematics_t::~robot_kinematics_t() {
}

Matrix4d robot_kinematics_t::legT(int _frame, double _u) {
	return _legT(_frame, _u + leg_u_off[_frame-1]);
}

Matrix4d robot_kinematics_t::legFK(const Vector6d& _u, bool _left) {
	return _legFK(_u, _left);
}

bool robot_kinematics_t::comIK(Skeleton *_robot,
							const Vector3d& _dcom,
							Matrix4d& _Twb,
							IK_Mode _mode[NUM_MANIPULATORS],
							const Matrix4d _Twm[NUM_MANIPULATORS],
							VectorXd& _dofs) {

	//TODO: Use COM Jacobians to turn the pelvis.
	// Or, pass in a rotated Twb and stanceIK to that.

	// Position robot in world frame
	_dofs.block<3,1>(0,0) = _Twb.block<3,1>(0,3);

	// Do gradient descent on COM error
	bool debug = false;
	int COM_ITER = 10000; // max iterations
	double COM_PTOL = 1e-3; // err tolerance
	double alpha = 0.5; // descent scaling factor
	bool ik_valid = true;
	bool ik_found = false;
	Vector3d com = Vector3d::Ones();
	Vector3d com_err;
	//TODO: Replace dof conversions with a dof class
	VectorXd u(24); // vector of manipulator joint angles
	for(int i=0; i < NUM_MANIPULATORS; ++i) {
		for(int j=0; j < 6; ++j) {
			int ind = dart_dof_ind[i][j];
			u(i*6 + j) = _dofs(ind); // init joint angles
		}
	}
	int i;
	for(i=0; i < COM_ITER; ++i) {
		// Solve for leg angles
		ik_valid = stanceIK(_Twb, _Twm[MANIP_L_FOOT], _Twm[MANIP_R_FOOT], u, u);

		// Position robot in world frame
		for(int i=0; i < NUM_MANIPULATORS; ++i) {
			for(int j=0; j < 6; ++j) {
				int ind = dart_dof_ind[i][j];
				_dofs(ind) = u(i*6 + j);
			}
		}
		_dofs.block<3,1>(0,0) = _Twb.block<3,1>(0,3);
		// Copy angles to skeleton
		_robot->setPose(_dofs, true, false);

		// COM in world frame
		com = _robot->getWorldCOM();

		// COM error
		com_err = _dcom - com;

		if(com_err.norm() < COM_PTOL) {
			ik_found = true;
			break;
		}

		// descent
		_Twb.block(0,3,3,1) += alpha * com_err;

		if(debug) {
			cout << "COM_IK: iteration " << i << endl;
			for(int i=0; i < NUM_MANIPULATORS; ++i) {
				cout << "manip " << i << "= " << u.block(i*6,0,6,1).transpose() << "\n";
			}
			cout << "dcom=\n" << _dcom << endl;
			cout << "com=\n" << com << endl;
			cout << endl;
		}
	}
	if(debug) {
		cout << "\nconverged at iteration " << i << "\n"
			 << "err= " << com_err.norm() << "\n\n";
	}

	return ik_found && ik_valid;
}

bool robot_kinematics_t::stanceIK(const Matrix4d& _Twb,
							   const Matrix4d& _Twl,
							   const Matrix4d& _Twr,
							   const VectorXd& _p,
							   VectorXd& _u) {
	Matrix4d Tbl = _Twb.inverse() * _Twl;
	Matrix4d Tbr = _Twb.inverse() * _Twr;

	Vector6d ul, ur;
	bool valid = legIK(Tbl, true, _p.block<6,1>(0,0), ul);
	valid &= legIK(Tbr, false, _p.block<6,1>(6,0), ur);
	_u.block<6,1>(0,0) = ul;
	_u.block<6,1>(6,0) = ur;

	return valid;
}

bool robot_kinematics_t::legIK(const Matrix4d& _Tbf, bool _left, const Vector6d& _p, Vector6d& _u) {
	MatrixXd tmp;
	return _legIK(_Tbf, _left, _p, true, _u, tmp);
}

Matrix4d robot_kinematics_t::_legT(int _f, double _u) {
	return dh2transform(leg_dh[_f-1].r, leg_dh[_f-1].a, leg_dh[_f].t+_u, leg_dh[_f].d);
}

Matrix4d robot_kinematics_t::_legFK(const Vector6d& _u, bool _left) {
	Matrix4d Tf, T01, T12, T23, T34, T45, T56;
	T01 = _legT(1, _u(0) + leg_u_off[0]);
	T12 = _legT(2, _u(1) + leg_u_off[1]);
	T23 = _legT(3, _u(2) + leg_u_off[2]);
	T34 = _legT(4, _u(3) + leg_u_off[3]);
	T45 = _legT(5, _u(4) + leg_u_off[4]);
	T56 = _legT(6, _u(5) + leg_u_off[5]);
	Tf = T01*T12*T23*T34*T45*T56;
	double l0 = leg_link_disp[0](1); // pelvis to hip
	Tf(1,3) += _left ? l0 : -l0;
	return Tf;
}

bool robot_kinematics_t::_legIK(Matrix4d _Tf,
							 bool _left,
							 const Vector6d& _p,
							 bool _nearest,
							 Vector6d& _u,
							 MatrixXd& _U) {
	// link lengths
	double l0 = leg_link_disp[0](1); // pelvis to hip
	double l1 = leg_link_disp[2](2); // hip yaw to hip pitch z
	double l2 = leg_link_disp[2](0); // hip yaw to hip pitch x
	double l3 = leg_link_disp[3](2); // hip pitch to knee pitch
	double l4 = leg_link_disp[4](2); // knee pitch to ankle pitch

	Matrix4d T0f = _Tf;
	T0f(1,3) += _left ? -l0 : l0;

	// % C++ implementation of script/AtlasLegIK.m
	// for convenience
	double x0,x1,x2,y0,y1,y2,z0,z1,z2,t0,t1,t2;
	x0 = T0f(0,0); y0 = T0f(0,1); z0 = T0f(0,2); t0 = T0f(0,3);
	x1 = T0f(1,0); y1 = T0f(1,1); z1 = T0f(1,2); t1 = T0f(1,3);
	x2 = T0f(2,0); y2 = T0f(2,1); z2 = T0f(2,2); t2 = T0f(2,3);

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
	double c4, cpsi;
	complex<double> radical;
	for(int i=0; i < 8; i++) {
		int i1 = m1[i];
		int i2 = m2[i];

		T01 = _legT(1, u1[i1]);
		T12 = _legT(2, u2[i2]);
		T2f = T12.inverse() * T01.inverse() * T0f;
		p2f = T2f.block(0,3,4,1);

		x = p2f(0) - l1;
		y = -p2f(2);

		c4 = (x*x + y*y - l3*l3 - l4*l4) / (2*l3*l4);
		radical = 1 - c4*c4;
		u4[i] = atan2( s4[i]*real(sqrt(radical)), c4 );

		//u4[i] = s4[i] * acos( c4 );

		phi = atan2( T2f(0,2), T2f(2,2) );
		beta = atan2(y,x);

		cpsi = (x*x + y*y + l3*l3 - l4*l4) / (2*l3*sqrt(x*x + y*y));
		radical = 1 - cpsi*cpsi;
		psi = atan2( real(sqrt(radical)), cpsi );

		//psi = acos( cpsi );

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

	// select solution
	MatrixXd U(6,8);
	bool within_lim[8] = {1,1,1,1,1,1,1,1};
	bool any_within = false;
	bool nearest_pos[8] = {0,0,0,0,0,0,0,0};
	double pos_delta[8];
	int within_count = 0;
	const double POS_TOL = 1e-10;
	const double JOINT_TOL = 1e-2;
	Matrix4d T06;
	// offset to joint zeros
	for(int i=0; i < 8; i++) {
		int i1 = m1[i];
		int i2 = m2[i];
		int i6 = m6[i];
		U(0,i) = u1[i1] - leg_u_off[0];
		U(1,i) = u2[i2] - leg_u_off[1];
		U(2,i) = u3[i] - leg_u_off[2];
		U(3,i) = u4[i] - leg_u_off[3];
		U(4,i) = u5[i] - leg_u_off[4];
		U(5,i) = u6[i6] - leg_u_off[5];
	}
	// clamp epsilon errors around joint limits
	// (for foot positions in workspace, but at joint limits)
	for(int i=0; i < 8; i++)
		for(int j=0; j < 6; j++)
			for(int k=0; k < 2; ++k)
				if(fabs(U(j,i) - leg_u_lim[j][k]) < JOINT_TOL)
					U(j,i) = leg_u_lim[j][k];
	// check joint limits
	for(int i=0; i < 8; i++) {
		for(int j=0; j < 6; j++)
			if(!(leg_u_lim[j][0] <= U(j,i) && U(j,i) <= leg_u_lim[j][1]))
				within_lim[i] = false;
		if(within_lim[i]) {
			any_within = true;
		}
	}
	// clamp to joint limits
	for(int i=0; i < 8; i++)
		for(int j=0; j < 6; j++) {
			if(U(j,i) < leg_u_lim[j][0])
				U(j,i) = leg_u_lim[j][0];
			if(leg_u_lim[j][1] < U(j,i))
				U(j,i) = leg_u_lim[j][1];
		}
	// complain if out of workspace
	if(!any_within) {
		cerr << "AtlasKinematics: IK Warning - Out of workspace\n"
			 << "left= " << _left << "\n"
		     << "T0f=\n" << T0f << "\n"
		     << "p=\n" << _p << "\n"
			 << "u=\n" << U << "\n"
			 << "\n";
	}
	if(!_nearest) {
		_U = U; // return clamped U
		return any_within; // complain if out of workspace
	}
	if(!any_within) {
		// no solutions within limits
		// find nearest(s) in position
		double min_delta = -1.0;
		for(int i=0; i < 8; i++) {
			T06 = _legFK(U.block<6,1>(0,i), _left);
			pos_delta[i] = (T06-_Tf).block<3,1>(0,3).norm();
			if(min_delta == -1.0 || pos_delta[i] < min_delta)
				min_delta = pos_delta[i];
		}
		for(int i=0; i < 8; i++) {
			if(fabs(min_delta - pos_delta[i]) < POS_TOL)
				nearest_pos[i] = true;
		}
	}
	// select closest to previous joint angles
	int sol = -1;
	double min_delta;
	for(int i=0; i < 8; i++) {
		if(within_lim[i] || nearest_pos[i]) {
			double delta = (U.block<6,1>(0,i) - _p).norm();
			if(sol == -1 || delta < min_delta) {
				sol = i;
				min_delta = delta;
			}
		}
	}
	_u = U.block<6,1>(0,sol);
	// complain about discontinuity
	if(false && min_delta >= JOINT_TOL) {
		cerr << "AtlasKinematics: IK Warning - Solution discontinuity\n"
			 << "T0f=\n" << T0f << "\n"
			 << "u delta=" << min_delta << "\n"
			 << "\n";
	}
	return any_within;
}

}
