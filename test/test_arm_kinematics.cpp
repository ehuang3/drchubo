#include <iostream>
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
Isometry3d Tw_msy; // World to shoulder frame 0 (for dart to align with DH)
/* ********************************************************************************************* */
void prepareWorld2DHShoulder() {
    Joint *arm_usy = _atlas->getJoint("l_arm_usy");
    Joint *arm_shx = _atlas->getJoint("l_arm_shx");
    Matrix4d shx = arm_shx->getTransform(0)->getTransform();
    Vector3d usy_axis = arm_usy->getAxis(0);
    Vector3d shx_disp = shx.block<3,1>(0,3);

//    cout << endl;
    
    double angle = -atan2(usy_axis(1), usy_axis(2)); //-30 angle
//    cout << "angle= " << angle << endl;

//    cout << "usy_axis = " << usy_axis.transpose() << endl;
//    cout << "shx_disp = " << shx_disp.transpose() << endl;

    Vector3d usy_off = usy_axis;
    usy_off *= usy_axis.dot(shx_disp);
//    cout << "usy_off = " << usy_off.transpose() << endl;
//    cout << "usy_off norm = " << usy_off.norm() << endl;

    Vector3d ssy_shx = shx_disp - usy_off;
//    cout << "ssy_shx = " << ssy_shx.transpose() << endl;
//    cout << "ssy_shx norm = " << ssy_shx.norm() << endl;

    // dsy = dh shoulder y
    Isometry3d Tusy_dsy;
    Tusy_dsy = Matrix4d::Identity();
    Tusy_dsy.rotate(AngleAxisd(angle, Vector3d::UnitX()));
    Tusy_dsy.translation() += usy_off;
//    cout << "Tusy_dsy = \n" << Tusy_dsy.matrix() << endl;
//    cout << endl;

    BodyNode *node_usy = arm_usy->getChildNode();
    
    Isometry3d Tw_dsy;
    Tw_dsy = node_usy->getWorldTransform();
    Tw_dsy = Tw_dsy * Tusy_dsy;
    
    BodyNode *node_shx = arm_shx->getChildNode();

    Isometry3d Tw_shx;
    Tw_shx = node_shx->getWorldTransform();
    
    Isometry3d Tdsy_shx;
    Tdsy_shx = Tw_dsy.inverse() * Tw_shx;
//    cout << "Tdsy_shx = \n" << Tdsy_shx.matrix() << endl;

    // msy = mock shoulder y (located on arm plane, behind shx)
    Tw_msy = Matrix4d::Identity();
    Isometry3d Tshx_msy;
    Tshx_msy = Matrix4d::Identity();
    Tshx_msy.translation() = Vector3d(0, -ssy_shx.norm(), 0); // move behind shx by correct ssy-shx disp
    Tw_msy = Tw_shx * Tshx_msy;
//    cout << "Tw_msy = \n" << Tw_msy.matrix() << endl;
//    cout << "Tw_shx = \n" << Tw_shx.matrix() << endl;
    

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
        prepareWorld2DHShoulder();
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
}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, FK_CMP_DART) {
    atlas_kinematics_t *ak = prepareAtlasKinematics();
    Isometry3d B;
    Matrix4d Bd;
    Vector6d q;

    // Compare 0 positions
    q.setZero();
    ak->_armFK(B, q, atlas_kinematics_t::SIDE_LEFT);
    cout << "B = \n" << B.matrix() << endl;
    
    Isometry3d Tmsy_hand;
    Tmsy_hand = Tw_msy.inverse() * node_hand->getWorldTransform();
    cout << "Tmsy_hand = \n" << Tmsy_hand.matrix() << endl;
}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, DART_LOCS) {
    BodyNode *l_clav = _atlas->getNode("l_clav");
    BodyNode *l_scap = _atlas->getNode("l_scap");
    BodyNode *l_uarm = _atlas->getNode("l_uarm");
    BodyNode *l_larm = _atlas->getNode("l_larm");
    BodyNode *l_farm = _atlas->getNode("l_farm");
    BodyNode *l_hand = _atlas->getNode("l_hand");

    auto clav_zero = l_clav->getWorldTransform().block<3,1>(0,3).transpose();

    Matrix4d Tmsy_shx, Tmsy_ely, Tmsy_elx, Tmsy_uwy, Tmsy_mwx;
    Matrix4d Tw_shx = l_scap->getWorldTransform();

    Tmsy_shx = Tw_shx.inverse() * l_scap->getWorldTransform();
    Tmsy_ely = Tw_shx.inverse() * l_uarm->getWorldTransform();
    Tmsy_elx = Tw_shx.inverse() * l_larm->getWorldTransform();
    Tmsy_uwy = Tw_shx.inverse() * l_farm->getWorldTransform();
    Tmsy_mwx = Tw_shx.inverse() * l_hand->getWorldTransform();
    
    cout << "Tmsy_shx = \n" << Tmsy_shx.matrix() << endl;
    cout << "Tmsy_ely = \n" << Tmsy_ely.matrix() << endl;
    cout << "Tmsy_elx = \n" << Tmsy_elx.matrix() << endl;
    cout << "Tmsy_uwy = \n" << Tmsy_uwy.matrix() << endl;
    cout << "Tmsy_mwx = \n" << Tmsy_mwx.matrix() << endl;

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
