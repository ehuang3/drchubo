#include "hubo_kinematics.h"
#include "hubo_state.h"

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

    void hubo_kinematics_t::xform_dh_wrist(Isometry3d& R, bool left)
    {
        R.matrix() << 
            -1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, -1, 0,
            0, 0, 0, 1;
    }

    void hubo_kinematics_t::xform_w_dsy(Isometry3d& B, bool left, robot::robot_state_t& state)
    {
        Eigen::VectorXd save = state.dart_pose();

        // Set arms to 0 so that we can grab the nominal shoulder transform
        VectorXd q(6);
        q.setZero();
        int mi = left ? robot::MANIP_L_HAND : robot::MANIP_R_HAND;
        state.set_manip(q, mi);

        Skeleton *hubo_skel = state.robot();
        hubo_skel->setPose(state.dart_pose());

        BodyNode* msr = hubo_skel->getNode(left?"Body_LSR":"Body_RSR");
        B = msr->getWorldTransform();

        // Undo zeroing of arm
        state.set_dart_pose(save);
    }

    void hubo_kinematics_t::init(Skeleton *_hubo) {
        robot = _hubo;

        // zero atlas
        VectorXd dofs = robot->getPose();
        VectorXd zerod_dofs(robot->getNumDofs());
        zerod_dofs.setZero();
        robot->setPose(zerod_dofs, true, false);

        hubo_state_t hubo_state;
        hubo_state.init(_hubo);

        vector<int> left_leg;
        vector<int> right_leg;

        hubo_state.get_manip_indexes(left_leg, robot::LIMB_L_LEG);
        hubo_state.get_manip_indexes(right_leg, robot::LIMB_R_LEG);

        BodyNode *left[6];
        BodyNode *right[6];

        for(int i=0; i < 6; i++) {
            left[i] = _hubo->getDof(left_leg[i])->getJoint()->getChildNode();
            right[i] = _hubo->getDof(right_leg[i])->getJoint()->getChildNode();
        }
    
        Matrix4d Tw1 = left[0]->getWorldTransform();
        Matrix4d Tw3 = left[2]->getWorldTransform();
        Matrix4d Tw4 = left[3]->getWorldTransform();
        Matrix4d Tw5 = left[4]->getWorldTransform();
        Matrix4d Tw6 = left[5]->getWorldTransform();

        double ox, oz;
        ox = abs(Tw1(0,3));
        oz = abs(Tw3(2,3));

        double l0, l1, l2, l3, l4;
        // double h3;
        l0 = abs(Tw6(1,3));
        l1 = 0;
        l2 = 0;
        l3 = abs(Tw4(2,3)) - oz;
        l4 = abs(Tw5(2,3)) - l3 - oz;

        // l3 = sqrt(h3*h3 + l2*l2);

        // angle offs
        for(int i=0; i < 6; ++i) {
            leg_u_off[i] = 0;
        }

        // // joint limits
        for(int i=0; i < 6; i++) {
            leg_u_lim[i][0] = left[i]->getParentJoint()->getDof(0)->getMin();
            leg_u_lim[i][1] = left[i]->getParentJoint()->getDof(0)->getMax();
        }

        // joint displacements
        // l0 is pelvis to hip
        leg_link_disp[0] = Vector3d(ox, l0, oz);
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

        //////////////////////////////////////////////////////////////
        /// CALCULATE ARM CONSTANTS FOR HUBO SOLVER
        //////////////////////////////////////////////////////////////
        BodyNode *LSP = robot->getNode("Body_LSP");
        BodyNode *LSR = robot->getNode("Body_LSR");
        BodyNode *LSY = robot->getNode("Body_LSY");
        BodyNode *LEP = robot->getNode("Body_LEP");
        BodyNode *LWY = robot->getNode("Body_LWY");
        BodyNode *LWP = robot->getNode("Body_LWP");

        BodyNode *links[] = { LSP, LSR, LSY, LEP, LWY, LWP };
        
        for(int i=0; i < 6; i++) {
            cout << links[i]->getName() << " = \n" << links[i]->getWorldTransform() << endl;
        }
        
        Matrix4d lsr = LSR->getWorldTransform();
        Matrix4d lep = LEP->getWorldTransform();
        Matrix4d lwp = LWP->getWorldTransform();

        arm.ssz = 0;
        arm.sez = fabs(lsr(2,3) - lep(2,3));
        arm.ewz = fabs(lep(2,3) - lwp(2,3));
        arm.whz = 0;

        // cheats!
        hubo_state.init(robot);

        vector<int> arm_indexes[2];
        vector<int> manip = { robot::MANIP_L_HAND, robot::MANIP_R_HAND };
        for(int i=0; i < 2; i++) {
            hubo_state.get_manip_indexes(arm_indexes[i], manip[i]);
        }
        
        for(int i=0; i<6; i++) {
            arm.left_limits.row(i) = hubo_state.get_limits(arm_indexes[0][i]).transpose();
        }
        for(int i=0; i<6; i++) {
            arm.right_limits.row(i) = hubo_state.get_limits(arm_indexes[1][i]).transpose();
        }
        
        arm.left_offset = Vector6d::Zero();
        arm.right_offset = Vector6d::Zero();
        
        arm.left_offset(2) = 0; //M_PI/2;
        arm.right_offset(2) = 0; //M_PI/2;
        
        arm.left_mirror << 1, 1, 1, 1, 1, 1;
        arm.right_mirror << 1, 1, 1, 1, 1, 1;
        
        //////////////////////////////////////////////////////////////
        /// CALCULATE ARM CONSTANTS FOR HUBO SOLVER
        //////////////////////////////////////////////////////////////
        // Joint *arm_usy = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][0])->getJoint();
        // Joint *arm_shx = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][1])->getJoint();
        // Joint *arm_ely = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][2])->getJoint();
        // Joint *arm_elx = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][3])->getJoint();
        // Joint *arm_uwy = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][4])->getJoint();
        // Joint *arm_mwx = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][5])->getJoint();

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

        // //////////////////////////////////////////////////////////////
        // /// ARM CONSTANTS
        // //////////////////////////////////////////////////////////////
        // // link lengths
        // //arm.ssz = dsy_shx_norm; //FIXME: USE CORRECT VALUE
        // arm.sez = fabs(ely(1,3) + elx(1,3));
        // arm.ewz = fabs(uwy(1,3) + mwx(1,3));
        // arm.whz = 0;

        // // DEBUG_PRINT("\n"
        // //             "arm nsy %f\n"
        // //             "arm ssz %f\n"
        // //             "arm sez %f\n"
        // //             "arm ewz %f\n",
        // //             arm.nsy, arm.ssz, arm.sez, arm.ewz);

        // // joint limits
        // for(int i=0; i < 6; i++) {
        //     arm.left_limits(i,0) = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][i])->getMin();
        //     arm.left_limits(i,1) = _atlas->getDof(dart_dof_ind[robot::MANIP_L_HAND][i])->getMax();
        // }
        // for(int i=0; i < 6; i++) {
        //     arm.right_limits(i,0) = _atlas->getDof(dart_dof_ind[robot::MANIP_R_HAND][i])->getMin();
        //     arm.right_limits(i,1) = _atlas->getDof(dart_dof_ind[robot::MANIP_R_HAND][i])->getMax();
        // }

        // // joint offsets << better read arm fk/ik implemenation to be consistent w/ mirrors
        // arm.left_offset = Vector6d::Zero();
        // arm.right_offset = Vector6d::Zero();
        // double shx_off = atan2(usy_axis(1), usy_axis(2));
        
        // arm.left_offset(1) = -shx_off;
        // arm.left_offset(2) = -M_PI/2;
        
        // arm.right_offset(1) = shx_off;
        // arm.right_offset(2) = M_PI/2;

        // // angle mirrors
        // arm.left_mirror << 1, 1, -1, 1, -1, 1;
        // arm.right_mirror << 1, 1, 1, 1, 1, 1;
    }

}
