#include "hubo_kinematics.h"

#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Transformation.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>

#include <iostream>
#include <stdio.h>

#define DEBUG
#define MODULE_NAME "HUBO-KIN"
#include "utils/debug_utils.h"

using namespace Eigen;
using namespace kinematics;
using namespace std;

namespace hubo {

hubo_kinematics_t::hubo_kinematics_t() {
}

hubo_kinematics_t::~hubo_kinematics_t() {
}

void hubo_kinematics_t::init(Skeleton *_hubo) {
	robot = _hubo;

	// // zero atlas
	// VectorXd dofs = _atlas->getPose();
	// VectorXd zerod_dofs(_atlas->getNumDofs());
	// zerod_dofs.setZero();
	// _atlas->setPose(zerod_dofs, true, false);

	// // joint 1 - hip yaw
	// BodyNode *LHY = _atlas->getNode("l_uglut");
	// // joint 2 - hip roll
	// BodyNode *LHR = _atlas->getNode("l_lglut");
	// // joint 3 - hip pitch
	// BodyNode *LHP = _atlas->getNode("l_uleg");
	// // joint 4 - knee pitch
	// BodyNode *LKP = _atlas->getNode("l_lleg");
	// // joint 5 - ankle pitch
	// BodyNode *LAP = _atlas->getNode("l_talus");
	// // joint 6 - ankle roll
	// BodyNode *LAR = _atlas->getNode("l_foot");

	// // BodyNode *P = _atlas->getNode("pelvis");

	// // "body origin" is at Atlas's pelvis
	// Matrix4d Tw1 = LHY->getWorldTransform();
	// Matrix4d Tw3 = LHP->getWorldTransform();
	// Matrix4d Tw4 = LKP->getWorldTransform();
	// Matrix4d Tw5 = LAP->getWorldTransform();

	// double l0, l1, l2, l3, l4;
	// double h3;
	// l0 = abs(Tw1(1,3));
	// l1 = abs(Tw3(2,3));
	// l2 = abs(Tw3(0,3));
	// h3 = abs(Tw4(2,3)) - l1; // l3 is at an angle
	// l4 = abs(Tw5(2,3)) - h3 - l1;

	// l3 = sqrt(h3*h3 + l2*l2);

	// // angle offs
	// for(int i=0; i < 6; ++i) {
	// 	leg_u_off[i] = 0;
	// }
	// leg_u_off[2] = atan2(l2, h3);
	// leg_u_off[3] = -leg_u_off[2];

	// // joint limits
	// BodyNode* node[6] = { LHY, LHR, LHP, LKP, LAP, LAR };
	// for(int i=0; i < 6; i++) {
	// 	leg_u_lim[i][0] = node[i]->getParentJoint()->getDof(0)->getMin();
	// 	leg_u_lim[i][1] = node[i]->getParentJoint()->getDof(0)->getMax();
	// }

	// // joint displacements
	// // l0 is pelvis to hip
	// leg_link_disp[0] = Vector3d(0, l0, 0);
	// leg_link_disp[1] = Vector3d(0, 0, 0);
	// // l1 is hip yaw to hip pitch z
	// // l2 is hip yaw to hip pitch x
	// leg_link_disp[2] = Vector3d(l2, 0, l1);
	// // l3 hip to knee
	// leg_link_disp[3] = Vector3d(0, 0, l3);
	// // l4 knee to ankle
	// leg_link_disp[4] = Vector3d(0, 0, l4);
	// leg_link_disp[5] = Vector3d(0, 0, 0);
	// leg_link_disp[6] = Vector3d(0, 0, 0);

	// // dh parameters
	// // frame 0 - hip origin with atlas xyz orientation
	// leg_dh[0].t = 0;
	// leg_dh[0].d = 0;
	// leg_dh[0].r = 0;
	// leg_dh[0].a = 0;
	// // frame 1 - hip yaw
	// leg_dh[1].t = M_PI/2;
	// leg_dh[1].d = 0;
	// leg_dh[1].r = 0;
	// leg_dh[1].a = M_PI/2;
	// // frame 2 - hip roll
	// leg_dh[2].t = -M_PI/2;
	// leg_dh[2].d = l2;
	// leg_dh[2].r = l1;
	// leg_dh[2].a = -M_PI/2;
	// // frame 3 - hip pitch
	// leg_dh[3].t = 0;
	// leg_dh[3].d = 0;
	// leg_dh[3].r = l3;
	// leg_dh[3].a = 0;
	// // frame 4 - knee pitch
	// leg_dh[4].t = 0;
	// leg_dh[4].d = 0;
	// leg_dh[4].r = l4;
	// leg_dh[4].a  = 0;
	// // frame 5 - ankle pitch
	// leg_dh[5].t = 0;
	// leg_dh[5].d = 0;
	// leg_dh[5].r  = 0;
	// leg_dh[5].a  = M_PI/2;
	// // frame 6 - ankle roll
	// leg_dh[6].t = 0;
	// leg_dh[6].d = 0;
	// leg_dh[6].r = 0;
	// leg_dh[6].a  = 0;

	// // index of joint angles in DART
	// dart_dof_ind[MANIP_L_FOOT][0] = 7;  //= l_leg_uhz
	// dart_dof_ind[MANIP_L_FOOT][1] = 10; //= l_leg_mhx
	// dart_dof_ind[MANIP_L_FOOT][2] = 13; //= l_leg_lhy
	// dart_dof_ind[MANIP_L_FOOT][3] = 18; //= l_leg_kny
	// dart_dof_ind[MANIP_L_FOOT][4] = 23; //= l_leg_uay
	// dart_dof_ind[MANIP_L_FOOT][5] = 27; //= l_leg_lax

	// dart_dof_ind[MANIP_R_FOOT][0] = 8;  //= r_leg_uhz
	// dart_dof_ind[MANIP_R_FOOT][1] = 11; //= r_leg_mhx
	// dart_dof_ind[MANIP_R_FOOT][2] = 14; //= r_leg_lhy
	// dart_dof_ind[MANIP_R_FOOT][3] = 19; //= r_leg_kny
	// dart_dof_ind[MANIP_R_FOOT][4] = 24; //= r_leg_uay
	// dart_dof_ind[MANIP_R_FOOT][5] = 28; //= r_leg_lax

	// dart_dof_ind[MANIP_L_HAND][0] = 15; //= l_arm_usy
	// dart_dof_ind[MANIP_L_HAND][1] = 20; //= l_arm_shx
	// dart_dof_ind[MANIP_L_HAND][2] = 25; //= l_arm_ely
	// dart_dof_ind[MANIP_L_HAND][3] = 29; //= l_arm_elx
	// dart_dof_ind[MANIP_L_HAND][4] = 31; //= l_arm_uwy
	// dart_dof_ind[MANIP_L_HAND][5] = 33; //= l_arm_mwx

	// dart_dof_ind[MANIP_R_HAND][0] = 17; //= r_arm_usy
	// dart_dof_ind[MANIP_R_HAND][1] = 22; //= r_arm_shx
	// dart_dof_ind[MANIP_R_HAND][2] = 26; //= r_arm_ely
	// dart_dof_ind[MANIP_R_HAND][3] = 30; //= r_arm_elx
	// dart_dof_ind[MANIP_R_HAND][4] = 32; //= r_arm_uwy
	// dart_dof_ind[MANIP_R_HAND][5] = 34; //= r_arm_mwx

	// // ARM
	// //	BodyNode *LSP = _atlas->getNode("Body_LSP");
	// //	BodyNode *LSR = _atlas->getNode("Body_LSR");
	// //	BodyNode *LSY = _atlas->getNode("Body_LSY");
	// //	BodyNode *LEP = _atlas->getNode("Body_LEP");
	// //	BodyNode *LWY = _atlas->getNode("Body_LWY");
	// //	BodyNode *LWP = _atlas->getNode("Body_LWP");

	// Joint *arm_usy = _atlas->getDof(dart_dof_ind[MANIP_L_HAND][0])->getJoint();
	// Joint *arm_shx = _atlas->getDof(dart_dof_ind[MANIP_L_HAND][1])->getJoint();
	// Joint *arm_ely = _atlas->getDof(dart_dof_ind[MANIP_L_HAND][2])->getJoint();
	// Joint *arm_elx = _atlas->getDof(dart_dof_ind[MANIP_L_HAND][3])->getJoint();
	// Joint *arm_uwy = _atlas->getDof(dart_dof_ind[MANIP_L_HAND][4])->getJoint();
	// Joint *arm_mwx = _atlas->getDof(dart_dof_ind[MANIP_L_HAND][5])->getJoint();

	// Matrix4d usy = arm_usy->getTransform(0)->getTransform();
	// Matrix4d shx = arm_shx->getTransform(0)->getTransform();
	// Matrix4d ely = arm_ely->getTransform(0)->getTransform();
    // Matrix4d elx = arm_elx->getTransform(0)->getTransform();
	// Matrix4d uwy = arm_uwy->getTransform(0)->getTransform();
	// Matrix4d mwx = arm_mwx->getTransform(0)->getTransform();

 	// // DEBUG_STREAM << "usy\n" << usy << endl;
	// // DEBUG_STREAM << "shx\n" << shx << endl;
	// // DEBUG_STREAM << "ely\n" << ely << endl;
	// // DEBUG_STREAM << "uwy\n" << uwy << endl;
    
    // // Get disp from dsy to shx
    // // dsy = DH shoulder y
    // Vector3d usy_axis = arm_usy->getAxis(0);
    // Vector3d shx_disp = shx.block<3,1>(0,3);
    // Vector3d usy_dsy_off = usy_axis * usy_axis.dot(shx_disp);
    // Vector3d dsy_shx_disp = shx_disp - usy_dsy_off;
    // double dsy_shx_norm = dsy_shx_disp.norm();

    // kc.arm_nsy = 0;
    // kc.arm_ssz = dsy_shx_norm;
    // kc.arm_sez = ely(1,3) + elx(1,3);
    // kc.arm_ewz = uwy(1,3) + mwx(1,3);
    // kc.arm_whz = 0;

    // // DEBUG_PRINT("\n"
    // //             "arm_nsy %f\n"
    // //             "arm_ssz %f\n"
    // //             "arm_sez %f\n"
    // //             "arm_ewz %f\n",
    // //             kc.arm_nsy, kc.arm_ssz, kc.arm_sez, kc.arm_ewz);
    
    // for(int i=0; i < 6; i++) {
    //     kc.arm_limits(i,0) = _atlas->getDof(dart_dof_ind[MANIP_L_HAND][i])->getMin();
    //     kc.arm_limits(i,1) = _atlas->getDof(dart_dof_ind[MANIP_L_HAND][i])->getMax();
    // }
    
    // // Joint offsets for zeroing into DH configuration
    // kc.arm_offset = Vector6d::Zero();
    // double shx_off = atan2(usy_axis(1), usy_axis(2)); //-30 angle
    // // double shx_off = -atan2(shx(1,3),shx(2,3));
    // DEBUG_PRINT("shx_off %f\n", shx_off);
    // kc.arm_offset(1) = shx_off;

    // kc.arm_mirror.push_back(1);
    // kc.arm_mirror.push_back(2);
    // kc.arm_mirror.push_back(4);

    // // Print out information about DART mappings
    // int manip_index[2] = { MANIP_L_HAND, MANIP_R_HAND };
    // for(int i=0; i < 2; i++) {
    //     // Print out limit information
    //     for(int j=0; j < 6; j++) {
    //         Dof *dof = _atlas->getDof(dart_dof_ind[manip_index[i]][j]);
    //         Joint *joint = dof->getJoint();
    //         BodyNode *node = joint->getChildNode();
    //         DEBUG_PRINT("Joint: %s limits %f to %f\n",
    //                     joint->getName(), dof->getMin(), dof->getMax());
    //     }
    // }
}

}
