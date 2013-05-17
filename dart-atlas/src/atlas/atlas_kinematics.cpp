#include "atlas_kinematics.h"

#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>

#include <iostream>

using namespace Eigen;
using namespace kinematics;
using namespace std;

namespace atlas {

atlas_kinematics_t::atlas_kinematics_t() {
}

atlas_kinematics_t::~atlas_kinematics_t() {
}

void atlas_kinematics_t::init(Skeleton *_atlas) {
	robot = _atlas;

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

	double l0, l1, l2, l3, l4;
	double h3;
	l0 = abs(Tw1(1,3));
	l1 = abs(Tw3(2,3));
	l2 = abs(Tw3(0,3));
	h3 = abs(Tw4(2,3)) - l1; // l3 is at an angle
	l4 = abs(Tw5(2,3)) - h3 - l1;

	l3 = sqrt(h3*h3 + l2*l2);

	// angle offs
	for(int i=0; i < 6; ++i) {
		leg_u_off[i] = 0;
	}
	leg_u_off[2] = atan2(l2, h3);
	leg_u_off[3] = -leg_u_off[2];

	// joint limits
	BodyNode* node[6] = { LHY, LHR, LHP, LKP, LAP, LAR };
	for(int i=0; i < 6; i++) {
		leg_u_lim[i][0] = node[i]->getParentJoint()->getDof(0)->getMin();
		leg_u_lim[i][1] = node[i]->getParentJoint()->getDof(0)->getMax();
	}

	// joint displacements
	// l0 is pelvis to hip
	leg_link_disp[0] = Vector3d(0, l0, 0);
	leg_link_disp[1] = Vector3d(0, 0, 0);
	// l1 is hip yaw to hip pitch z
	// l2 is hip yaw to hip pitch x
	leg_link_disp[2] = Vector3d(l2, 0, l1);
	// l3 hip to knee
	leg_link_disp[3] = Vector3d(0, 0, l3);
	// l4 knee to ankle
	leg_link_disp[4] = Vector3d(0, 0, l4);
	leg_link_disp[5] = Vector3d(0, 0, 0);
	leg_link_disp[6] = Vector3d(0, 0, 0);

	// dh parameters
	// frame 0 - hip origin with atlas xyz orientation
	leg_dh[0].t = 0;
	leg_dh[0].d = 0;
	leg_dh[0].r = 0;
	leg_dh[0].a = 0;
	// frame 1 - hip yaw
	leg_dh[1].t = M_PI/2;
	leg_dh[1].d = 0;
	leg_dh[1].r = 0;
	leg_dh[1].a = M_PI/2;
	// frame 2 - hip roll
	leg_dh[2].t = -M_PI/2;
	leg_dh[2].d = l2;
	leg_dh[2].r = l1;
	leg_dh[2].a = -M_PI/2;
	// frame 3 - hip pitch
	leg_dh[3].t = 0;
	leg_dh[3].d = 0;
	leg_dh[3].r = l3;
	leg_dh[3].a = 0;
	// frame 4 - knee pitch
	leg_dh[4].t = 0;
	leg_dh[4].d = 0;
	leg_dh[4].r = l4;
	leg_dh[4].a  = 0;
	// frame 5 - ankle pitch
	leg_dh[5].t = 0;
	leg_dh[5].d = 0;
	leg_dh[5].r  = 0;
	leg_dh[5].a  = M_PI/2;
	// frame 6 - ankle roll
	leg_dh[6].t = 0;
	leg_dh[6].d = 0;
	leg_dh[6].r = 0;
	leg_dh[6].a  = 0;

	// index of joint angles in DART
	dart_dof_ind[MANIP_L_FOOT][0] = 7;  //= l_leg_uhz
	dart_dof_ind[MANIP_L_FOOT][1] = 10; //= l_leg_mhx
	dart_dof_ind[MANIP_L_FOOT][2] = 13; //= l_leg_lhy
	dart_dof_ind[MANIP_L_FOOT][3] = 18; //= l_leg_kny
	dart_dof_ind[MANIP_L_FOOT][4] = 23; //= l_leg_uay
	dart_dof_ind[MANIP_L_FOOT][5] = 27; //= l_leg_lax

	dart_dof_ind[MANIP_R_FOOT][0] = 8;  //= r_leg_uhz
	dart_dof_ind[MANIP_R_FOOT][1] = 11; //= r_leg_mhx
	dart_dof_ind[MANIP_R_FOOT][2] = 14; //= r_leg_lhy
	dart_dof_ind[MANIP_R_FOOT][3] = 19; //= r_leg_kny
	dart_dof_ind[MANIP_R_FOOT][4] = 24; //= r_leg_uay
	dart_dof_ind[MANIP_R_FOOT][5] = 28; //= r_leg_lax

	dart_dof_ind[MANIP_L_HAND][0] = 15; //= l_arm_usy
	dart_dof_ind[MANIP_L_HAND][1] = 20; //= l_arm_shx
	dart_dof_ind[MANIP_L_HAND][2] = 25; //= l_arm_ely
	dart_dof_ind[MANIP_L_HAND][3] = 29; //= l_arm_elx
	dart_dof_ind[MANIP_L_HAND][4] = 31; //= l_arm_uwy
	dart_dof_ind[MANIP_L_HAND][5] = 33; //= l_arm_mwx

	dart_dof_ind[MANIP_R_HAND][0] = 17; //= r_arm_usy
	dart_dof_ind[MANIP_R_HAND][1] = 22; //= r_arm_shx
	dart_dof_ind[MANIP_R_HAND][2] = 26; //= r_arm_ely
	dart_dof_ind[MANIP_R_HAND][3] = 30; //= r_arm_elx
	dart_dof_ind[MANIP_R_HAND][4] = 32; //= r_arm_uwy
	dart_dof_ind[MANIP_R_HAND][5] = 34; //= r_arm_mwx
}

}
