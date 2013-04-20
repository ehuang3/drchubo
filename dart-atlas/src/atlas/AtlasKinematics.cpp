#include "AtlasKinematics.h"
#include "AtlasUtils.h"

#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>

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
		atlas = _atlas;

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

		// BodyNode *P = _atlas->getNode("pelvis");

		// "body origin" is at Atlas's pelvis
		Matrix4d Tw1 = LHY->getWorldTransform();
		Matrix4d Tw3 = LHP->getWorldTransform();
		Matrix4d Tw4 = LKP->getWorldTransform();
		Matrix4d Tw5 = LAP->getWorldTransform();

		double h3;
		l0 = abs(Tw1(1,3));
		l1 = abs(Tw3(2,3));
		l2 = abs(Tw3(0,3));
		h3 = abs(Tw4(2,3)) - l1; // l3 is at an angle
		l4 = abs(Tw5(2,3)) - h3 - l1;

		l3 = sqrt(h3*h3 + l2*l2);

		// angle offs
		for(int i=0; i < 6; ++i) {
			u_off[i] = 0;
		}
		u_off[2] = atan2(l2, h3);
		u_off[3] = -u_off[2];

		// joint limits
		BodyNode* node[6] = { LHY, LHR, LHP, LKP, LAP, LAR };
		for(int i=0; i < 6; i++) {
			u_lim[i][0] = node[i]->getParentJoint()->getDof(0)->getMin();
			u_lim[i][1] = node[i]->getParentJoint()->getDof(0)->getMax();
			// cout << "u_lim["<<i<<"][0] = " << u_lim[i][0] << ";" << endl;
			// cout << "u_lim["<<i<<"][1] = " << u_lim[i][1] << ";" << endl;
		}
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

Matrix4d AtlasKinematics::legFK(const Vector6d& _u, bool _left) {
	return _legFK(_u, _left);
}

bool AtlasKinematics::comIK(Skeleton *_atlas,
							const Vector3d& _dcom,
							Matrix4d& _Twb,
							IK_Mode _mode[NUM_MANIPULATORS],
							const Matrix4d _Twm[NUM_MANIPULATORS],
							VectorXd& _dofs) {
	// joint indexes in dofs
	vector<int> dof_ind[NUM_MANIPULATORS];
	for(int i=0; i < NUM_MANIPULATORS; i++) {
		for(int j=0; j < 6; j++) {
			dof_ind[i].push_back(0); // allocate space
		}
	}
	dof_ind[MANIP_L_FOOT][0] = 7;  //= l_leg_uhz
	dof_ind[MANIP_L_FOOT][1] = 10; //= l_leg_mhx
	dof_ind[MANIP_L_FOOT][2] = 13; //= l_leg_lhy
	dof_ind[MANIP_L_FOOT][3] = 18; //= l_leg_kny
	dof_ind[MANIP_L_FOOT][4] = 23; //= l_leg_uay
	dof_ind[MANIP_L_FOOT][5] = 27; //= l_leg_lax

	dof_ind[MANIP_R_FOOT][0] = 8;  //= r_leg_uhz
	dof_ind[MANIP_R_FOOT][1] = 11; //= r_leg_mhx
	dof_ind[MANIP_R_FOOT][2] = 14; //= r_leg_lhy
	dof_ind[MANIP_R_FOOT][3] = 19; //= r_leg_kny
	dof_ind[MANIP_R_FOOT][4] = 24; //= r_leg_uay
	dof_ind[MANIP_R_FOOT][5] = 28; //= r_leg_lax

	dof_ind[MANIP_L_HAND][0] = 15; //= l_arm_usy
	dof_ind[MANIP_L_HAND][1] = 20; //= l_arm_shx
	dof_ind[MANIP_L_HAND][2] = 25; //= l_arm_ely
	dof_ind[MANIP_L_HAND][3] = 29; //= l_arm_elx
	dof_ind[MANIP_L_HAND][4] = 31; //= l_arm_uwy
	dof_ind[MANIP_L_HAND][5] = 33; //= l_arm_mwx

	dof_ind[MANIP_R_HAND][0] = 17; //= r_arm_usy
	dof_ind[MANIP_R_HAND][1] = 22; //= r_arm_shx
	dof_ind[MANIP_R_HAND][2] = 26; //= r_arm_ely
	dof_ind[MANIP_R_HAND][3] = 30; //= r_arm_elx
	dof_ind[MANIP_R_HAND][4] = 32; //= r_arm_uwy
	dof_ind[MANIP_R_HAND][5] = 34; //= r_arm_mwx

	//	[] = 0; //= floatingX
	//	[] = 1; //= floatingY
	//	[] = 2; //= floatingZ
	//	[] = 3; //= floatingYaw
	//	[] = 4; //= floatingPitch
	//	[] = 5; //= floatingRoll
	//	[] = 6; //= back_lbz
	//	[] = 9; //= back_mby
	//	[] = 12; //= back_ubx
	//	[] = 16; //= neck_ay
	//	[] = 21; //= hokuyo_joint

	// hack
	_dofs.block(0,0,3,1) = Vector3d::Zero();

	// gradient descent on com err
	bool debug = false;
	int COM_ITER = 10000; // max iterations
	double COM_PTOL = 1e-3; // err tolerance
	double alpha = 0.5; // descent scaling factor
	bool ik_valid = true;
	bool ik_found = false;
	Vector4d com = Vector4d::Ones();
	Vector3d com_err;
	VectorXd u(24); // vector of manipulator joint angles
	for(int i=0; i < NUM_MANIPULATORS; ++i) {
		for(int j=0; j < 6; ++j) {
			int ind = dof_ind[i][j];
			u(i*6 + j) = _dofs(ind); // init joint angles
		}
	}
	int i;
	for(i=0; i < COM_ITER; ++i) {
		// solve for leg angles
		ik_valid = stanceIK(_Twb, _Twm[MANIP_L_FOOT], _Twm[MANIP_R_FOOT], u, u);

		// copy angles to skeleton
		for(int i=0; i < NUM_MANIPULATORS; ++i) {
			for(int j=0; j < 6; ++j) {
				int ind = dof_ind[i][j];
				_dofs(ind) = u(i*6 + j);
			}
		}
		_atlas->setPose(_dofs, true, false);

		// com in body frame
		com.block<3,1>(0,0) = _atlas->getWorldCOM();
		// translate to world frame
		com = _Twb * com;
		// error
		com_err = _dcom - com.block<3,1>(0,0);

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

	// so we can visualize correctly later :)
	_dofs.block(0,0,3,1) = _Twb.block(0,3,3,1);

	return ik_found && ik_valid;
}

bool AtlasKinematics::stanceIK(const Matrix4d& _Twb,
							   const Matrix4d& _Twl,
							   const Matrix4d& _Twr,
							   const VectorXd& _p,
							   VectorXd& _u) {
	Matrix4d Tbl = _Twb.inverse() * _Twl;
	Matrix4d Tbr = _Twb.inverse() * _Twr;

	Vector6d ul, ur;
	bool valid = legIK(Tbl, true, _p.block<6,1>(0,0), ul) &&
				 legIK(Tbr, false, _p.block<6,1>(6,0), ur);
	_u.block<6,1>(0,0) = ul;
	_u.block<6,1>(6,0) = ur;

	return valid;
}

bool AtlasKinematics::legIK(const Matrix4d& _Tbf, bool _left, const Vector6d& _p, Vector6d& _u) {
	MatrixXd tmp;
	return _legIK(_Tbf, _left, _p, true, _u, tmp);
}

bool AtlasKinematics::legIK(const Matrix4d& _Tbf, bool _left, MatrixXd& _u) {
	Vector6d tmp;
	return _legIK(_Tbf, _left, Vector6d::Zero(), false, tmp, _u);
}

Matrix4d AtlasKinematics::_legT(int _f, double _u) {
	return dh2transform(lr[_f-1], la[_f-1], lt[_f]+_u, ld[_f]);
}

Matrix4d AtlasKinematics::_legFK(const Vector6d& _u, bool _left) {
	Matrix4d Tf, T01, T12, T23, T34, T45, T56;
	T01 = _legT(1, _u(0) + u_off[0]);
	T12 = _legT(2, _u(1) + u_off[1]);
	T23 = _legT(3, _u(2) + u_off[2]);
	T34 = _legT(4, _u(3) + u_off[3]);
	T45 = _legT(5, _u(4) + u_off[4]);
	T56 = _legT(6, _u(5) + u_off[5]);
	Tf = T01*T12*T23*T34*T45*T56;
	Tf(1,3) += _left ? l0 : -l0;
	return Tf;
}

//TODO: workspace limits
bool AtlasKinematics::_legIK(Matrix4d _Tf,
							 bool _left,
							 const Vector6d& _p,
							 bool _nearest,
							 Vector6d& _u,
							 MatrixXd& _U) {
	// undo body to hip translation
	_Tf(1,3) += _left ? -l0 : l0;

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

	// fill solutions
	MatrixXd U(6,8);
	bool valid[8] = {1,1,1,1,1,1,1,1};
	int valid_count = 0;
	const double JOINT_TOL = 1e-10;
	for(int i=0; i < 8; i++) {
		int i1 = m1[i];
		int i2 = m2[i];
		int i6 = m6[i];
		// offset to joint zeros
		U(0,i) = u1[i1] - u_off[0];
		U(1,i) = u2[i2] - u_off[1];
		U(2,i) = u3[i] - u_off[2];
		U(3,i) = u4[i] - u_off[3];
		U(4,i) = u5[i] - u_off[4];
		U(5,i) = u6[i6] - u_off[5];
		for(int j=0; j < 6; ++j) {
			// clamp small values around joint limits to joint limits
			// (epsilon errors affect joint limit selection)
			for(int k=0; k < 2; ++k) {
				if(abs(U(j,i) - u_lim[j][k]) < JOINT_TOL) {
					U(j,i) = u_lim[j][k];
				}
			}
			// check joint limits
			if( !(u_lim[j][0] <= U(j,i) &&
				       U(j,i) <= u_lim[j][1])) {
				valid[i] = false;
			}
			// check nan solutions
			//TODO: handle nans by clamping bad input to atan/acos above
			if( std::isnan(U(j,i))) {
				valid[i] = false;
			}
		}
		if(valid[i])
			valid_count++;
	}
	// no solution found
	if(valid_count == 0) {
		cerr << "AtlasKinematics: IK Error - No solution found!\n"
		     << "T0f=\n" << _Tf << "\n"
			 << "u=\n" << U << "\n"
			 << "\n";
		//FIXME: return solution closest to Tf instead
		_u = _p; // return original angles (on _nearest)
		return false;
	}

	if(_nearest) {
		// return nearest solution to _p
		double min_dist;
		bool min_init = false;
		for(int i=0; i < 8; ++i) {
			if(valid[i]) {
				double dist = (U.block(0,i,6,1) - _p).norm();
				if(!min_init || dist < min_dist) {
					min_init = true;
					min_dist = dist;
					_u = U.block(0,i,6,1);
				}
			}
		}
	} else {
		// return all valid solutions
		_U.resize(6, valid_count);
		int v = 0;
		for(int i=0; i < 8; ++i) {
			if(valid[i]) {
				_U.block(0,v++,6,1) = U.block(0,i,6,1);
			}
		}
	}
	return true;
}

}
