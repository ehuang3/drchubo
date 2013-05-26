#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
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
#include <kinematics/Joint.h>
#include <kinematics/Transformation.h>
#include <dynamics/SkeletonDynamics.h>

using namespace std;
using namespace Eigen;
using namespace atlas;
using namespace robot;

using namespace kinematics;
using namespace dynamics;
using namespace simulation;

atlas::atlas_kinematics_t *_ak;
kinematics::Skeleton *_atlas;
kinematics::BodyNode *node_hand;
kinematics::BodyNode *node_clav;
// Isometry3d Tw_dsy; // World to DH shoulder frame 0
// Isometry3d Tw_msy; // World to shoulder frame 0 mocked (for quick testing link lengths)
/* ********************************************************************************************* */
void DART_ZEROD_FK(Isometry3d& B, const Vector6d& q, int side) {
    bool left = side == robot::SIDE_LEFT;
    int manip_index = left ? robot_kinematics_t::MANIP_L_HAND : robot_kinematics_t::MANIP_R_HAND;
    VectorXd dofs = _atlas->getPose();
    for(int i=0; i < 6; i++) {
        // Mirror for right arm
        if(!left && (i == 1 || i == 3 || i == 5)) {
            dofs(_ak->dart_dof_ind[manip_index][i]) = -q(i);
        } else {
            dofs(_ak->dart_dof_ind[manip_index][i]) = q(i);
        }
        // Offset joint to DH zero config
        if(i == 1) {
            Vector3d usy_axis = _atlas->getJoint(left ? "l_arm_usy" : "r_arm_usy")->getAxis(0);
            // atan2 sends right arm 180 back
            double shx_off = (left?0:M_PI) + atan2(usy_axis(1), usy_axis(2)); // -30 degrees
            dofs(_ak->dart_dof_ind[manip_index][i]) -= shx_off;
        }
    }
    _atlas->setPose(dofs, true);
    B = _atlas->getNode(left ? "l_hand" : "r_hand")->getWorldTransform();
}
/* ********************************************************************************************* */
const int TEST_RIGHT = 0;
const int TEST_LEFT = 1;
void XFORM_W_DSY(Isometry3d& B, int side) {
    bool left = side == TEST_LEFT; //< Needs to be consistent w/ robot_kinematics_t
    Joint *arm_usy = _atlas->getJoint(left?"l_arm_usy":"r_arm_usy");
    Joint *arm_shx = _atlas->getJoint(left?"l_arm_shx":"r_arm_shx");
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
    // cout << "Tusy_dsy = \n" << Tusy_dsy.matrix() << endl;
    // Transform w to dsy
    Isometry3d Tw_usy;
    Tw_usy = arm_usy->getChildNode()->getWorldTransform();
    Isometry3d Tw_dsy = Tw_usy * Tusy_dsy;
    // Return
    B = Tw_dsy;
}
/* ********************************************************************************************* */
struct atlas_constants_t {
    double r_shx_angle;
    double l_shx_angle;
    double usy_dsy_norm;
    double dsy_shx_norm;
    double shx_elx_norm;
    double elx_mwx_norm;
    double dsy_mwx_norm;
    
    void print() {
        printf("r_shx_angle = %f\n"
               "l_shx_angle = %f\n"
               "usy_dsy_norm = %f\n"
               "dsy_shx_norm = %f\n"
               "shx_elx_norm = %f\n"
               "elx_mwx_norm = %f\n"
               "dsy_mwx_norm = %f\n",
               r_shx_angle, l_shx_angle, usy_dsy_norm, dsy_shx_norm,
               shx_elx_norm, elx_mwx_norm, dsy_mwx_norm);
    }
};
void ATLAS_CONSTANTS(atlas_constants_t& ac) {
    // Zero atlas
    _atlas->setPose(_atlas->getPose().setZero(), true);
    // Calculate distances
    Joint *arm_usy = _atlas->getJoint("l_arm_usy");
    Joint *arm_shx = _atlas->getJoint("l_arm_shx");
    Matrix4d shx = arm_shx->getTransform(0)->getTransform();
    // shx_disp = usy to shx offset (dart)
    Vector3d usy_axis = arm_usy->getAxis(0);
    Vector3d shx_disp = shx.block<3,1>(0,3);
    // Offset to 0 arm at joint shx
    double angle = atan2(usy_axis(1), usy_axis(2)); //-30 angle
    // cout << "shx angle = " << angle << endl;
    ac.l_shx_angle = angle;
    ac.r_shx_angle = -angle;
    // Vector from usy to dsy
    Vector3d usy_dsy = usy_axis;
    usy_dsy *= usy_axis.dot(shx_disp);
    ac.usy_dsy_norm = usy_dsy.norm();
    // Vector dsy to shx
    Vector3d dsy_shx = shx_disp - usy_dsy;
    // cout << "dsy_shx norm = " << dsy_shx.norm() << endl;
    ac.dsy_shx_norm = dsy_shx.norm();
    
    // Find link lengths using msy.
    // msy = mock shoulder y (located on arm plane, behind shx)
    BodyNode *node_shx = arm_shx->getChildNode();
    Isometry3d Tw_shx;
    Tw_shx = node_shx->getWorldTransform();
    Isometry3d Tw_msy;
    Tw_msy = Matrix4d::Identity();
    Isometry3d Tshx_msy;
    Tshx_msy = Matrix4d::Identity();
    Tshx_msy.translation() = Vector3d(0, -dsy_shx.norm(), 0); // move behind shx by correct ssy-shx disp
    Tw_msy = Tw_shx * Tshx_msy;

    // Link lengths
    BodyNode *l_clav = _atlas->getNode("l_clav");
    BodyNode *l_scap = _atlas->getNode("l_scap");
    BodyNode *l_uarm = _atlas->getNode("l_uarm");
    BodyNode *l_larm = _atlas->getNode("l_larm");
    BodyNode *l_farm = _atlas->getNode("l_farm");
    BodyNode *l_hand = _atlas->getNode("l_hand");

    Tw_shx = l_scap->getWorldTransform();
    Matrix4d Tw_elx = l_larm->getWorldTransform();
    Matrix4d Tw_mwx = l_hand->getWorldTransform();

    ac.shx_elx_norm = (Tw_shx.inverse() * Tw_elx)(1,3);
    ac.elx_mwx_norm = (Tw_elx.inverse() * Tw_mwx)(1,3);
    ac.dsy_mwx_norm = (Tw_msy.inverse() * Tw_mwx)(1,3);
}
/* ********************************************************************************************* */
atlas::atlas_kinematics_t *prepareAtlasKinematics() {
	if(!_ak) {
		DartLoader dart_loader;
		World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH "models/atlas/atlas_world.urdf");
		_atlas = mWorld->getSkeleton("atlas");
		_ak = new atlas_kinematics_t();
		_ak->init(_atlas);
        node_hand = _atlas->getNode("l_hand");
        node_clav = _atlas->getNode("l_clav");

        atlas_constants_t ac;
        ATLAS_CONSTANTS(ac);
        ac.print();
	}
	_atlas->setPose(_atlas->getPose().setZero(), true);
	return _ak;
}
/* ********************************************************************************************* */
TEST(KINEMATICS, INIT) {
	atlas_kinematics_t *AK = prepareAtlasKinematics();
}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, DART_JOINT_OFFSETS) {
    // Determine if joint offsets zero atlas out correctly in DART
    Vector6d q;
    q.setZero();
    Isometry3d Tw_mwx;
    DART_ZEROD_FK(Tw_mwx, q, robot::SIDE_LEFT);
    Isometry3d Tw_dsy;
    XFORM_W_DSY(Tw_dsy, TEST_LEFT);
    Isometry3d Tdsy_mwx = Tw_dsy.inverse() * Tw_mwx;
    cout << "Joint offsets put LEFT Tdsy_mwx at = \n" << Tdsy_mwx.matrix() << endl;
    // Location of first joint w/ joint offsets
    // Assume dofs have been set correctly in atlas from before
    Isometry3d Tw_shx;
    Tw_shx = _atlas->getJoint("l_arm_shx")->getChildNode()->getWorldTransform();
    Isometry3d Tdsy_shx = Tw_dsy.inverse() * Tw_shx;
    // cout << "Joint offsets put LEFT Tdsy_shx at = \n" << Tdsy_shx.matrix() << endl;
    // Determine if joint offsets zero RIGHT side correctly
    DART_ZEROD_FK(Tw_mwx, q, robot::SIDE_RIGHT);
    XFORM_W_DSY(Tw_dsy, TEST_RIGHT);
    Tdsy_mwx = Tw_dsy.inverse() * Tw_mwx;
    cout << "Joint offsets put RIGHT Tdsy_mwx at = \n" << Tdsy_mwx.matrix() << endl;
    Tw_shx = _atlas->getJoint("r_arm_shx")->getChildNode()->getWorldTransform();
    Tdsy_shx = Tw_dsy.inverse() * Tw_shx;
    // cout << "Joint offsets put RIGHT Tdsy_shx at = \n" << Tdsy_shx.matrix() << endl;
    // Determine what a positive angle is for joints
    q << 0, 0, 0, M_PI/2, 0, 0; // Should rotate elx
    DART_ZEROD_FK(Tw_mwx, q, TEST_LEFT);
    XFORM_W_DSY(Tw_dsy, TEST_LEFT);
    Tdsy_mwx = Tw_dsy.inverse() * Tw_mwx;
    cout << "90 elx puts LEFT Tdsy_mwx at = \n" << Tdsy_mwx.matrix() << endl;
    // now right
    DART_ZEROD_FK(Tw_mwx, q, TEST_RIGHT);
    XFORM_W_DSY(Tw_dsy, TEST_RIGHT);
    Tdsy_mwx = Tw_dsy.inverse() * Tw_mwx;
    cout << "90 elx puts RIGHT Tdsy_mwx at = \n" << Tdsy_mwx.matrix() << endl;
}
/* ********************************************************************************************* */
TEST(KINEMATICS, FORWARD) {
	atlas_kinematics_t *AK = prepareAtlasKinematics();
	Vector6d u = Vector6d::Zero();
}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, FK_CMP_DART) {
    atlas_kinematics_t *ak = prepareAtlasKinematics();
    Isometry3d B;
    Matrix4d Bd;
    Vector6d q;

    Isometry3d Bdsy_mwx;
    
    // Compare 0 positions
//     q.setZero();
//     ak->_armFK(Bdsy_mwx, q, robot::SIDE_LEFT);
//     cout << "Bdsy_mwx = \n" << Bdsy_mwx.matrix() << endl;
    
//     Isometry3d Tw_msy;
//     Isometry3d Tmsy_hand;
//     Tmsy_hand = Tw_msy.inverse() * node_hand->getWorldTransform();
//     cout << "Tmsy_hand = \n" << Tmsy_hand.matrix() << endl;

//     q << 0, M_PI/2, 0, 0, 0, 0;
//     ak->_armFK(B, q, robot::SIDE_LEFT);
// //    ak->dart_armFK(Tmsy_hand, q, robot_kinematics_t::SIDE_LEFT);
//     Tmsy_hand = Tw_msy.inverse() * Tmsy_hand;
//     cout << "B = \n" << B.matrix() << endl;
//     cout << "Tmsy_hand = \n" << Tmsy_hand.matrix() << endl;
}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, DART_LOCS) {
    BodyNode *l_clav = _atlas->getNode("l_clav");
    BodyNode *l_scap = _atlas->getNode("l_scap");
    BodyNode *l_uarm = _atlas->getNode("l_uarm");
    BodyNode *l_larm = _atlas->getNode("l_larm");
    BodyNode *l_farm = _atlas->getNode("l_farm");
    BodyNode *l_hand = _atlas->getNode("l_hand");

//    auto clav_zero = l_clav->getWorldTransform().block<3,1>(0,3).transpose();

    Matrix4d Tmsy_shx, Tmsy_ely, Tmsy_elx, Tmsy_uwy, Tmsy_mwx;
    Matrix4d Tw_shx = l_scap->getWorldTransform();

    Tmsy_shx = Tw_shx.inverse() * l_scap->getWorldTransform();
    Tmsy_ely = Tw_shx.inverse() * l_uarm->getWorldTransform();
    Tmsy_elx = Tw_shx.inverse() * l_larm->getWorldTransform();
    Tmsy_uwy = Tw_shx.inverse() * l_farm->getWorldTransform();
    Tmsy_mwx = Tw_shx.inverse() * l_hand->getWorldTransform();
    
    // cout << "Tmsy_shx = \n" << Tmsy_shx.matrix() << endl;
    // cout << "Tmsy_ely = \n" << Tmsy_ely.matrix() << endl;
    // cout << "Tmsy_elx = \n" << Tmsy_elx.matrix() << endl;
    // cout << "Tmsy_uwy = \n" << Tmsy_uwy.matrix() << endl;
    // cout << "Tmsy_mwx = \n" << Tmsy_mwx.matrix() << endl;

    // cout << "l_clav= " << l_clav->getWorldTransform().block<3,1>(0,3).transpose() - clav_zero << endl;
    // cout << "l_scap= " << l_scap->getWorldTransform().block<3,1>(0,3).transpose() - clav_zero << endl;
    // cout << "l_uarm= " << l_uarm->getWorldTransform().block<3,1>(0,3).transpose() - clav_zero << endl;
    // cout << "l_larm= " << l_larm->getWorldTransform().block<3,1>(0,3).transpose() - clav_zero << endl;
    // cout << "l_farm= " << l_farm->getWorldTransform().block<3,1>(0,3).transpose() - clav_zero << endl;
    // cout << "l_hand= " << l_hand->getWorldTransform().block<3,1>(0,3).transpose() - clav_zero << endl;
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
