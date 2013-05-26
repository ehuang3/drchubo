#include "atlas_kinematics.h"

#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Transformation.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>

#include <iostream>
#include <stdio.h>

#define DEBUG
#define MODULE_NAME "ATLAS-KIN"
#include "utils/debug_utils.h"
#include "robot/robot_state.h"

using namespace Eigen;
using namespace kinematics;
using namespace std;
using namespace robot;

namespace atlas {

atlas_kinematics_t::atlas_kinematics_t() {
}

atlas_kinematics_t::~atlas_kinematics_t() {
}

    void atlas_kinematics_t::xform_dh_wrist(Isometry3d& R)
    {
        // from dh convention to atlas convention
        R.matrix() << 
            0, 1, 0, 0,
            1, 0, 0, 0,
            0, 0, -1, 0,
            0, 0, 0, 1;
    }

    void atlas_kinematics_t::xform_w_dsy(Isometry3d& B, bool left, robot_state_t& state)
    {
        // Transform magic, I'd rather not explain unless I'm held down and threatened with a knife.
        // Need to orient a frame from world to DH origin.
        Skeleton *atlas = state.robot();
        state.copy_into_robot();
        Joint *arm_usy = atlas->getJoint(left?"l_arm_usy":"r_arm_usy");
        Joint *arm_shx = atlas->getJoint(left?"l_arm_shx":"r_arm_shx");
        Matrix4d shx = arm_shx->getTransform(0)->getTransform();
        Vector3d usy_axis = arm_usy->getAxis(0);
        Vector3d shx_disp = shx.block<3,1>(0,3);
        // Offset to 0 arm at joint shx
        double angle = -atan2(usy_axis(1), usy_axis(2)); //-30 angle
        // Vector from usy to dsy
        Vector3d usy_dsy = usy_axis;
        usy_dsy *= usy_axis.dot(shx_disp);
        // Vector dsy to shx
        Vector3d dsy_shx = shx_disp - usy_dsy;
        // Transform usy to dsy
        Isometry3d Tusy_dsy;
        Tusy_dsy = Matrix4d::Identity();
        Tusy_dsy.rotate(AngleAxisd(angle, Vector3d::UnitX()));
        Tusy_dsy.translation() += usy_dsy;
        Tusy_dsy.translation() += dsy_shx; //FIXME: REMOVE!
        // cout << "Tusy_dsy = \n" << Tusy_dsy.matrix() << endl;
        // Transform w to dsy
        Isometry3d Tw_usy;
        Tw_usy = arm_usy->getChildNode()->getWorldTransform();
        Isometry3d Tw_body;
        state.get_body(Tw_body);
        Isometry3d Tbody_usy = Tw_body.inverse() * Tw_usy;
        // the angle of usy will induce a rotation on Tw_usy
        // DH origin has no rotation, so we must kill the induced rotation
        Tbody_usy.linear() = Matrix3d::Identity();
        Isometry3d Tw_dsy = Tw_body * Tbody_usy * Tusy_dsy;
        // Rotate axis to match DH
        // The dsy I calculated above points its y axis down the arm and that's incorrect.
        // The DH origin is oriented in the same as DARTs standard axes
        // We convert w_dsy back into the standard axes
        Matrix3d Rw_dh;
        Rw_dh << 
            1, 0, 0,
            0, 0, -1,
            0, 1, 0;
        Tw_dsy.linear() = Tw_dsy.linear() * Rw_dh;
        // Return
        B = Tw_dsy;
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
	dart_dof_ind[robot::MANIP_L_FOOT][0] = 7;  //= l_leg_uhz
	dart_dof_ind[robot::MANIP_L_FOOT][1] = 10; //= l_leg_mhx
	dart_dof_ind[robot::MANIP_L_FOOT][2] = 13; //= l_leg_lhy
	dart_dof_ind[robot::MANIP_L_FOOT][3] = 18; //= l_leg_kny
	dart_dof_ind[robot::MANIP_L_FOOT][4] = 23; //= l_leg_uay
	dart_dof_ind[robot::MANIP_L_FOOT][5] = 27; //= l_leg_lax

	dart_dof_ind[robot::MANIP_R_FOOT][0] = 8;  //= r_leg_uhz
	dart_dof_ind[robot::MANIP_R_FOOT][1] = 11; //= r_leg_mhx
	dart_dof_ind[robot::MANIP_R_FOOT][2] = 14; //= r_leg_lhy
	dart_dof_ind[robot::MANIP_R_FOOT][3] = 19; //= r_leg_kny
	dart_dof_ind[robot::MANIP_R_FOOT][4] = 24; //= r_leg_uay
	dart_dof_ind[robot::MANIP_R_FOOT][5] = 28; //= r_leg_lax

	dart_dof_ind[robot::MANIP_L_HAND][0] = 15; //= l_arm_usy
	dart_dof_ind[robot::MANIP_L_HAND][1] = 20; //= l_arm_shx
	dart_dof_ind[robot::MANIP_L_HAND][2] = 25; //= l_arm_ely
	dart_dof_ind[robot::MANIP_L_HAND][3] = 29; //= l_arm_elx
	dart_dof_ind[robot::MANIP_L_HAND][4] = 31; //= l_arm_uwy
	dart_dof_ind[robot::MANIP_L_HAND][5] = 33; //= l_arm_mwx

	dart_dof_ind[robot::MANIP_R_HAND][0] = 17; //= r_arm_usy
	dart_dof_ind[robot::MANIP_R_HAND][1] = 22; //= r_arm_shx
	dart_dof_ind[robot::MANIP_R_HAND][2] = 26; //= r_arm_ely
	dart_dof_ind[robot::MANIP_R_HAND][3] = 30; //= r_arm_elx
	dart_dof_ind[robot::MANIP_R_HAND][4] = 32; //= r_arm_uwy
	dart_dof_ind[robot::MANIP_R_HAND][5] = 34; //= r_arm_mwx

	// ARM
	//	BodyNode *LSP = _atlas->getNode("Body_LSP");
	//	BodyNode *LSR = _atlas->getNode("Body_LSR");
	//	BodyNode *LSY = _atlas->getNode("Body_LSY");
	//	BodyNode *LEP = _atlas->getNode("Body_LEP");
	//	BodyNode *LWY = _atlas->getNode("Body_LWY");
	//	BodyNode *LWP = _atlas->getNode("Body_LWP");

    ////////////////////////////////////////////////////////////////////////////
    /// CALCULATE ARM CONSTANTS FOR HUBO SOLVER
    ////////////////////////////////////////////////////////////////////////////
	Joint *arm_usy = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][0])->getJoint();
	Joint *arm_shx = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][1])->getJoint();
	Joint *arm_ely = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][2])->getJoint();
	Joint *arm_elx = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][3])->getJoint();
	Joint *arm_uwy = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][4])->getJoint();
	Joint *arm_mwx = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][5])->getJoint();

	Matrix4d usy = arm_usy->getTransform(0)->getTransform();
	Matrix4d shx = arm_shx->getTransform(0)->getTransform();
	Matrix4d ely = arm_ely->getTransform(0)->getTransform();
    Matrix4d elx = arm_elx->getTransform(0)->getTransform();
	Matrix4d uwy = arm_uwy->getTransform(0)->getTransform();
	Matrix4d mwx = arm_mwx->getTransform(0)->getTransform();

 	// DEBUG_STREAM << "usy\n" << usy << endl;
	// DEBUG_STREAM << "shx\n" << shx << endl;
	// DEBUG_STREAM << "ely\n" << ely << endl;
	// DEBUG_STREAM << "uwy\n" << uwy << endl;
    
    // Get disp from dsy to shx
    // dsy = DH shoulder y
    Vector3d usy_axis = arm_usy->getAxis(0);
    Vector3d shx_disp = shx.block<3,1>(0,3);
    Vector3d usy_dsy_off = usy_axis * usy_axis.dot(shx_disp);
    Vector3d dsy_shx_disp = shx_disp - usy_dsy_off;
    double dsy_shx_norm = dsy_shx_disp.norm();

    robot_arm_constants_t rac;

    rac.arm_nsy = 0;
    rac.arm_ssz = 0; //dsy_shx_norm; //FIXME: USE CORRECT VALUE
    rac.arm_sez = fabs(ely(1,3) + elx(1,3));
    rac.arm_ewz = fabs(uwy(1,3) + mwx(1,3));
    rac.arm_whz = 0;

    DEBUG_PRINT("\n"
                "arm_nsy %f\n"
                "arm_ssz %f\n"
                "arm_sez %f\n"
                "arm_ewz %f\n",
                rac.arm_nsy, rac.arm_ssz, rac.arm_sez, rac.arm_ewz);
    
    for(int i=0; i < 6; i++) {
        rac.arm_limits(i,0) = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][i])->getMin();
        rac.arm_limits(i,1) = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][i])->getMax();
    }
    
    // We need to offset joint angles to nominal zero positions that DART has
    // Shoulder roll needs to be 30 away from the body and shoulder yaw needs to 
    // turn his hand so that elbow and wrist pitches are such that positive angles
    // (in HUBO DH world) drives the arm into the body.
    rac.arm_offset = Vector6d::Zero();
    // 30 degree angle between DH vertical axis and nominal zero arm position
    double shx_off = atan2(usy_axis(1), usy_axis(2));
    DEBUG_PRINT("shx_off %f\n", shx_off);
    // note that offsets turn the arm opposite of what a joint angle value does
    rac.arm_offset(1) = -shx_off; //< this will turn left arm up by 30 degrees 
    rac.arm_offset(2) = M_PI/2; //< rotate elbow to open upwards

    rac.arm_mirror.push_back(1);
    rac.arm_mirror.push_back(3);
    rac.arm_mirror.push_back(5);

    // Dart to DH positive angle convention
    int d2dh_map[] = { 1, 1, -1, 1, -1, 1 }; // 1 agrees w/ dart, -1 swap w/ dart
    // flip limits to respect positive angle convention
    for(int i=0; i < 6; i++) {
        if(d2dh_map[i] == -1) {
            std::swap(rac.arm_limits(i,0), rac.arm_limits(i,1));
            rac.arm_limits(i,0) *= d2dh_map[i];
            rac.arm_limits(i,1) *= d2dh_map[i];
        }
    }

    rak.set_constants(rac);
}

}
