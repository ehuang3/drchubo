#include "test_utils.h"
#include <robot/robot_arm_kinematics.h>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace Eigen;

using namespace kinematics;
using namespace simulation;
using namespace dynamics;

using namespace robot;
using namespace atlas;
Matrix4d zero_it(Isometry3d B, double tol = 1e-12)
{
    for(int i=0; i < 4; i++) 
        for(int j=0; j < 4; j++)
            if(fabs(B(i,j)) < tol)
                B(i,j) = 0;
    return B.matrix();
}
/* ********************************************************************************************* */
TEST(ATLAS_ARM, PRINT_LEFT_CONSTANTS)
{
    robot_state_t *state = PREPARE_ROBOT_STATE();
    robot_kinematics_t *kin = PREPARE_ROBOT_KINEMATICS();
    
    vector<int> left_arm;
    state->get_manip_indexes(left_arm, MANIP_L_HAND);
    state->print_limits(left_arm);

    vector<int> right_arm;
    state->get_manip_indexes(right_arm, MANIP_R_HAND);
    state->print_limits(right_arm);
}
/* ********************************************************************************************* */
TEST(ATLAS_ARM, PROTO_LEFT_XFORM_W_DSY)
{
    // PREPARE TEST
    robot_state_t* state = PREPARE_ROBOT_STATE();
    robot_kinematics_t *kin = PREPARE_ROBOT_KINEMATICS();
    robot_arm_kinematics_t akin = kin->arm_kin();
    vector<int> left_arm;
    state->dofs().setZero();
    state->get_manip_indexes(left_arm, MANIP_L_HAND);
    VectorXd q(6);
    q.setZero();
    q << 0, -M_PI/6, 0, 0, 0, 0;
    state->set_dofs(q, left_arm);
    bool left = true;

    // XFORM_W_DSY CODE
    Skeleton *atlas = state->robot();
    state->copy_into_robot();
    Joint *arm_usy = atlas->getJoint(left?"l_arm_usy":"r_arm_usy");
    Joint *arm_shx = atlas->getJoint(left?"l_arm_shx":"r_arm_shx");
    Matrix4d shx = arm_shx->getTransform(0)->getTransform();
    Vector3d usy_axis = arm_usy->getAxis(0);
    Vector3d shx_disp = shx.block<3,1>(0,3);
    // Offset to 0 arm at joint shx
    double angle = -atan2(usy_axis(1), usy_axis(2)); //-30 angle
    cout << "angle = " << angle << endl;
    // Vector from usy to dsy
    Vector3d usy_dsy = usy_axis;
    usy_dsy *= usy_axis.dot(shx_disp);
    cout << "usy_dsy = " << usy_dsy.transpose() << endl;
    // Vector dsy to shx
    Vector3d dsy_shx = shx_disp - usy_dsy;
    cout << "dsy_shx = " << dsy_shx.transpose() << endl;
    // Transform usy to dsy
    Isometry3d Tusy_dsy;
    Tusy_dsy = Matrix4d::Identity();
    Tusy_dsy.rotate(AngleAxisd(angle, Vector3d::UnitX()));
    Tusy_dsy.translation() += usy_dsy;
    Tusy_dsy.translation() += dsy_shx; //FIXME: REMOVE!
    cout << "Tusy_dsy = \n" << Tusy_dsy.matrix() << endl;
    // Transform w to dsy
    Isometry3d Tw_usy;
    Tw_usy = arm_usy->getChildNode()->getWorldTransform();
    Isometry3d Tw_body;
    state->get_body(Tw_body);
    Isometry3d Tbody_usy = Tw_body.inverse() * Tw_usy;
    // DH origin has no rotation, so we much kill it here
    Tbody_usy.linear() = Matrix3d::Identity();
    Isometry3d Tw_dsy = Tw_body * Tbody_usy * Tusy_dsy;

    // VERIFY 
    Isometry3d Tw_hand;
    Tw_hand = state->robot()->getNode(ROBOT_LEFT_HAND)->getWorldTransform();

    // actually, i don't know how to test this here. TEST_LEFT_FK will garuentee the correctness
    // basically, Tw_dsy should NOT have any rotation from usy. It needs to be in the origin location.

    printf("Tw_dsy=\n");
    cout << zero_it(Tw_dsy) << endl;
    
    printf("Tdsy_hand=\n");
    cout << zero_it(Tw_dsy.inverse() * Tw_hand) << endl;

}
/* ********************************************************************************************* */
TEST(ATLAS_ARM, PROTO_LEFT_FK)
{
    robot_state_t *state = PREPARE_ROBOT_STATE();
    robot_kinematics_t *kin = PREPARE_ROBOT_KINEMATICS();
    robot_arm_kinematics_t akin = kin->arm_kin();
    
    vector<int> left_arm;
    state->get_manip_indexes(left_arm, MANIP_L_HAND);

    // Setup joint angles
    VectorXd q(6);
    Vector6d q6;
    q6.setZero();
    q.setZero();
    
    // q6 << M_PI/2, 0, 0, M_PI/2, 0, 0;
    // q6 << 0, M_PI/6, 0, 0, 0, 0;
    // q6 << 0.14, 1, 0.153, 1.231, -0.44, 0.5;
    q = q6;
    
    // convert to dh convention...
    // q6 = -q6;
    q6(2) = -q6(2);
    q6(4) = -q6(4);
    
    // world xform
    state->set_dofs(q, left_arm);
    state->copy_into_robot();

    Isometry3d Tw_hand;
    Tw_hand = state->robot()->getNode(ROBOT_LEFT_HAND)->getWorldTransform();

    // HUBOFK xform
    Isometry3d B;
    akin.armFK(B, q6, SIDE_LEFT);

    // xform to world
    Isometry3d Tw_dsy;
    kin->xform_w_dsy(Tw_dsy, true, *state);

    // xform hand to dart
    Isometry3d Tdh_wrist;
    kin->xform_dh_wrist(Tdh_wrist);

    // print
    printf("HUBOFK xform B=\n");
    cout << zero_it(B*Tdh_wrist) << endl;

    printf("ATLAS Tw_hand =\n");
    cout << zero_it(Tw_hand) << endl;

    printf("ATLAS Tdsy_hand =\n");
    Isometry3d Tdsy_hand = Tw_dsy.inverse() * Tw_hand;
    cout << zero_it(Tdsy_hand) << endl;

    printf("HUBOFK Tw_B=\n");
    cout << zero_it(Tw_dsy * B * Tdh_wrist) << endl;

    ASSERT_MATRIX_EQ((Tw_dsy*B*Tdh_wrist).rotation(), Tw_hand.rotation(), 1e-7);
    ASSERT_MATRIX_EQ((Tw_dsy*B*Tdh_wrist).translation(), Tw_hand.translation(), 0.03);
}
/* ********************************************************************************************* */
TEST(ATLAS_ARM, PROTO_LEFT_IK)
{
    robot_state_t *state = PREPARE_ROBOT_STATE();
    robot_kinematics_t *kin = PREPARE_ROBOT_KINEMATICS();
    robot_arm_kinematics_t akin = kin->arm_kin();
    
    vector<int> left_arm;
    state->get_manip_indexes(left_arm, MANIP_L_HAND);

    // Setup joint angles
    VectorXd q(6);
    Vector6d q6;
    q6.setZero();
    q.setZero();

    //q6 << M_PI/2, 0, M_PI/2, M_PI/2, 0, 0;
    q6 << 0.14, 1, 0.153, 1.231, -0.44, 0.5;
    //q6 << 0, 0, 0, 0, 0, 0;
    q = q6;
    
    // convert to dh convention...
    // q6 = -q6;
    q6(2) = -q6(2);
    q6(4) = -q6(4);
    
    // Setup world xform + joint angles
    state->set_dofs(q, left_arm);
    state->copy_into_robot();

    // Verify that HUBO FK/IK is setup consistently
    // xform to HUBOFK
    Isometry3d B;
    akin.armFK(B, q6, SIDE_LEFT);

    // HUBOIK verify q6
    Vector6d qik;
    akin.armIK(qik, B, q6, SIDE_LEFT);
    printf("q6 =\n");
    cout << q6 << endl;

    printf("qik =\n");
    cout << qik << endl;

    Isometry3d B1, B2;
    akin.armFK(B1, q6, SIDE_LEFT);
    akin.armFK(B2, qik, SIDE_LEFT);

    printf("B1(q6)=\n");
    cout << zero_it(B1) << endl;

    printf("B2(qik)=\n");
    cout << zero_it(B2) << endl;

    ASSERT_MATRIX_EQ(q6, qik);

    // Test Dart -> Hubo IK
    
    // q -> Dart hand
    Isometry3d Tw_hand;
    Tw_hand = state->robot()->getNode(ROBOT_LEFT_HAND)->getWorldTransform();
    
    // xform to world
    Isometry3d Tw_dsy;
    kin->xform_w_dsy(Tw_dsy, true, *state);
    // xform dh hand to dart
    Isometry3d Tdh_wrist;
    kin->xform_dh_wrist(Tdh_wrist);
    // Map hand into DH space
    Isometry3d Bw;
    Bw = Tw_dsy.inverse() * Tw_hand * Tdh_wrist.inverse();

    // B and Bw should be almost exactly close
    printf("B=\n");
    cout << zero_it(B) << endl;
    printf("Bw=\n");
    cout << zero_it(Bw) << endl;

    // HUBO IK
    akin.armIK(qik, Bw, q6, SIDE_LEFT);

    // save it
    // convert to dart
    qik(2) *= -1;
    qik(4) *= -1;

    state->set_dofs(qik, left_arm);
    state->copy_into_robot();

    // print
    printf("q6 =\n");
    cout << q6 << endl;

    printf("qik =\n");
    cout << qik << endl;

    Isometry3d Tw_qik;
    Tw_qik = state->robot()->getNode(ROBOT_LEFT_HAND)->getWorldTransform();

    printf("Tw_hand=\n");
    cout << zero_it(Tw_hand) << endl;

    printf("Tw_qik=\n");
    cout << zero_it(Tw_qik) << endl;

    // Isometry3d ans;
    // akin.armFK(ans, q6, SIDE_LEFT);
    // cout << "q6 FK=\n" << zero_it(ans) << endl;
    
    // akin.armFK(ans, qik, SIDE_LEFT);
    // cout << "qik FK=\n" << zero_it(ans) << endl;
}
/* ********************************************************************************************* */
TEST(ATLAS_ARM, PROTO_RIGHT_XFORM_W_DSY)
{
    // PREPARE TEST
    robot_state_t* state = PREPARE_ROBOT_STATE();
    robot_kinematics_t *kin = PREPARE_ROBOT_KINEMATICS();
    robot_arm_kinematics_t akin = kin->arm_kin();
    vector<int> right_arm;
    state->dofs().setZero();
    state->get_manip_indexes(right_arm, MANIP_R_HAND);
    VectorXd q(6);
    q.setZero();
    q << 0, M_PI/6, 0, 0, 0, 0;
    state->set_dofs(q, right_arm);
    bool left = false;

    // XFORM_W_DSY CODE
    Skeleton *atlas = state->robot();
    state->copy_into_robot();
    Joint *arm_usy = atlas->getJoint(left?"l_arm_usy":"r_arm_usy");
    Joint *arm_shx = atlas->getJoint(left?"l_arm_shx":"r_arm_shx");
    Matrix4d shx = arm_shx->getTransform(0)->getTransform();
    Vector3d usy_axis = arm_usy->getAxis(0);
    Vector3d shx_disp = shx.block<3,1>(0,3);
    // Offset to 0 arm at joint shx
    double angle = -atan2(usy_axis(1), usy_axis(2)); //-30 angle for left
    cout << "angle = " << angle << endl;
    // Vector from usy to dsy
    Vector3d usy_dsy = usy_axis;
    usy_dsy *= usy_axis.dot(shx_disp);
    cout << "usy_dsy = " << usy_dsy.transpose() << endl;
    // Vector dsy to shx
    Vector3d dsy_shx = shx_disp - usy_dsy;
    cout << "dsy_shx = " << dsy_shx.transpose() << endl;
    // Transform usy to dsy
    Isometry3d Tusy_dsy;
    Tusy_dsy = Matrix4d::Identity();
    Tusy_dsy.rotate(AngleAxisd(angle, Vector3d::UnitX()));
    Tusy_dsy.translation() += usy_dsy;
    Tusy_dsy.translation() += dsy_shx; //FIXME: REMOVE!
    cout << "Tusy_dsy = \n" << Tusy_dsy.matrix() << endl;
    // Transform w to dsy
    Isometry3d Tw_usy;
    Tw_usy = arm_usy->getChildNode()->getWorldTransform();
    Isometry3d Tw_body;
    state->get_body(Tw_body);
    Isometry3d Tbody_usy = Tw_body.inverse() * Tw_usy;
    // DH origin has no rotation, so we much kill it here
    Tbody_usy.linear() = Matrix3d::Identity();
    Isometry3d Tw_dsy = Tw_body * Tbody_usy * Tusy_dsy;

    // VERIFY 
    Isometry3d Tw_hand;
    Tw_hand = state->robot()->getNode(ROBOT_RIGHT_HAND)->getWorldTransform();

    // actually, i don't know how to test this here. TEST_RIGHT_FK will garuentee the correctness
    // basically, Tw_dsy should NOT have any rotation from usy. It needs to be in the origin location.

    printf("Tw_dsy=\n");
    cout << zero_it(Tw_dsy) << endl;
    
    printf("Tdsy_hand=\n");
    cout << zero_it(Tw_dsy.inverse() * Tw_hand) << endl;

}
/* ********************************************************************************************* */
TEST(ATLAS_ARM, PROTO_RIGHT_FK)
{
    robot_state_t *state = PREPARE_ROBOT_STATE();
    robot_kinematics_t *kin = PREPARE_ROBOT_KINEMATICS();
    robot_arm_kinematics_t akin = kin->arm_kin();
    
    vector<int> right_arm;
    state->get_manip_indexes(right_arm, MANIP_R_HAND);

    // Setup joint angles
    VectorXd q(6);
    Vector6d q6;
    q6.setZero();
    q.setZero();
    
    // q6 << M_PI/2, 0, 0, M_PI/2, 0, 0;
    // q6 << 0, M_PI/6, 0, 0, 0, 0;
    // q6 << 0.14, 1, 0.153, 1.231, -0.44, 0.5;
    q = q6;
    
    // convert to dh convention...
    // q6 = -q6;
    q6(2) = -q6(2);
    q6(4) = -q6(4);
    
    // world xform
    state->set_dofs(q, right_arm);
    state->copy_into_robot();

    Isometry3d Tw_hand;
    Tw_hand = state->robot()->getNode(ROBOT_RIGHT_HAND)->getWorldTransform();

    // HUBOFK xform
    Isometry3d B;
    akin.armFK(B, q6, SIDE_RIGHT);

    // xform to world
    Isometry3d Tw_dsy;
    kin->xform_w_dsy(Tw_dsy, false, *state);

    // xform hand to dart
    Isometry3d Tdh_wrist;
    kin->xform_dh_wrist(Tdh_wrist);

    // print
    printf("HUBOFK xform B=\n");
    cout << zero_it(B*Tdh_wrist) << endl;

    printf("ATLAS Tw_hand =\n");
    cout << zero_it(Tw_hand) << endl;

    printf("ATLAS Tdsy_hand =\n");
    Isometry3d Tdsy_hand = Tw_dsy.inverse() * Tw_hand;
    cout << zero_it(Tdsy_hand) << endl;

    printf("HUBOFK Tw_B=\n");
    cout << zero_it(Tw_dsy * B * Tdh_wrist) << endl;

    ASSERT_MATRIX_EQ((Tw_dsy*B*Tdh_wrist).rotation(), Tw_hand.rotation(), 1e-7);
    ASSERT_MATRIX_EQ((Tw_dsy*B*Tdh_wrist).translation(), Tw_hand.translation(), 0.03);
}
/* ********************************************************************************************* */
TEST(ATLAS_ARM, PROTO_RIGHT_IK)
{
    robot_state_t *state = PREPARE_ROBOT_STATE();
    robot_kinematics_t *kin = PREPARE_ROBOT_KINEMATICS();
    robot_arm_kinematics_t akin = kin->arm_kin();
    
    vector<int> right_arm;
    state->get_manip_indexes(right_arm, MANIP_R_HAND);

    state->print_limits(right_arm);

    // Setup joint angles
    VectorXd q(6);
    Vector6d q6;
    q6.setZero();
    q.setZero();

    //q6 << M_PI/2, 0, M_PI/2, M_PI/2, 0, 0;
    q6 << 0.14, 1, -0.153, 1.231, -0.44, 0.5;
    //q6 << 0, 0, 0, 0, 0, 0;
    q = q6;
    
    // convert to dh convention...
    // q6 = -q6;
    q6(2) = -q6(2);
    q6(4) = -q6(4);
    
    // Setup world xform + joint angles
    state->set_dofs(q, right_arm);
    state->copy_into_robot();

    // Verify that HUBO FK/IK is setup consistently
    // xform to HUBOFK
    Isometry3d B;
    akin.armFK(B, q6, SIDE_RIGHT);

    // HUBOIK verify q6
    Vector6d qik;
    akin.armIK(qik, B, q6, SIDE_RIGHT);
    // printf("q6 =\n");
    // cout << q6 << endl;

    // printf("qik =\n");
    // cout << qik << endl;

    Isometry3d B1, B2;
    akin.armFK(B1, q6, SIDE_RIGHT);
    akin.armFK(B2, qik, SIDE_RIGHT);

    // printf("B1(q6)=\n");
    // cout << zero_it(B1) << endl;

    // printf("B2(qik)=\n");
    // cout << zero_it(B2) << endl;

    ASSERT_MATRIX_EQ(q6, qik);

    // Test Dart -> Hubo IK
    
    // q -> Dart hand
    Isometry3d Tw_hand;
    Tw_hand = state->robot()->getNode(ROBOT_RIGHT_HAND)->getWorldTransform();
    
    // xform to world
    Isometry3d Tw_dsy;
    kin->xform_w_dsy(Tw_dsy, false, *state);
    // xform dh hand to dart
    Isometry3d Tdh_wrist;
    kin->xform_dh_wrist(Tdh_wrist);
    // Map hand into DH space
    Isometry3d Bw;
    Bw = Tw_dsy.inverse() * Tw_hand * Tdh_wrist.inverse();

    // B and Bw should be almost exactly close
    // printf("B=\n");
    // cout << zero_it(B) << endl;
    // printf("Bw=\n");
    // cout << zero_it(Bw) << endl;

    // HUBO IK
    akin.armIK(qik, Bw, q6, SIDE_RIGHT);

    // save it
    // convert to dart
    qik(2) *= -1;
    qik(4) *= -1;

    state->set_dofs(qik, right_arm);
    state->copy_into_robot();

    // print
    // printf("q6 =\n");
    // cout << q6 << endl;

    // printf("qik =\n");
    // cout << qik << endl;

    // Isometry3d Tw_qik;
    // Tw_qik = state->robot()->getNode(ROBOT_RIGHT_HAND)->getWorldTransform();

    // printf("Tw_hand=\n");
    // cout << zero_it(Tw_hand) << endl;

    // printf("Tw_qik=\n");
    // cout << zero_it(Tw_qik) << endl;

    // Isometry3d ans;
    // akin.armFK(ans, q6, SIDE_RIGHT);
    // cout << "q6 FK=\n" << zero_it(ans) << endl;
    
    // akin.armFK(ans, qik, SIDE_RIGHT);
    // cout << "qik FK=\n" << zero_it(ans) << endl;
}
/* ********************************************************************************************* */
TEST(ATLAS_ARM, TEST_RIGHT_IK)
{
    robot_state_t *state = PREPARE_ROBOT_STATE();
    robot_kinematics_t *kin = PREPARE_ROBOT_KINEMATICS();
    robot_arm_kinematics_t akin = kin->arm_kin();
    
    vector<int> arm;
    state->get_manip_indexes(arm, MANIP_R_HAND);

    // Setup joint angles
    VectorXd q(6);
    Vector6d q6;
    q6.setZero();
    q.setZero();

    //q6 << M_PI/2, 0, M_PI/2, M_PI/2, 0, 0;
    // q6 << 0.14, 1, 0.153, 1.231, -0.44, 0.5;

    q6 << 0, 0, 0, 0, 0, 0;
    q = q6;

    // Setup world xform + joint angles
    state->set_dofs(q, arm);
    state->copy_into_robot();

    // q -> Dart hand
    Isometry3d Tw_hand;
    Tw_hand = state->robot()->getNode(ROBOT_RIGHT_HAND)->getWorldTransform();

    // arm ik
    state->dofs().setZero();
    kin->arm_ik(Tw_hand, false, *state);
    
    // get solution
    state->get_manip(q, MANIP_R_HAND);
    
    // print
    // cout << "q6 = \n" << q6 << endl;

    // cout << "qsol = \n" << q << endl;
}
/* ********************************************************************************************* */
TEST(ATLAS_ARM, TEST_LEFT_JOINT_DIRECTIONS)
{
    robot_kinematics_t *kin = PREPARE_ROBOT_KINEMATICS();
    robot_arm_kinematics_t akin = kin->arm_kin();

    Vector6d q6;
    q6.setZero();

    Vector6d qz;
    qz << 0, -M_PI/6, 0, 0, 0, 0;
    
    Isometry3d B;

    akin.armFK(B, qz+q6, SIDE_LEFT);
    
    cout << "q is zero, B = \n" << zero_it(B) << endl;

    double val = M_PI/2;
    for(int i=0; i < 6; ++i) {
        q6.setZero();
        q6(i) = val;
        akin.armFK(B, qz+q6, SIDE_LEFT);
        printf("q(%d) = %f, B = \n", i, val);
        cout << zero_it(B) << endl;
    }
}
/* ********************************************************************************************* */
TEST(ATLAS_ARM, TEST_RIGHT_JOINT_DIRECTIONS)
{
    robot_kinematics_t *kin = PREPARE_ROBOT_KINEMATICS();
    robot_arm_kinematics_t akin = kin->arm_kin();

    Vector6d q6;
    q6.setZero();

    Vector6d qz;
    qz << 0, M_PI/6, 0, 0, 0, 0;
    
    Isometry3d B;

    akin.armFK(B, qz+q6, SIDE_RIGHT);
    
    cout << "q is zero, B = \n" << zero_it(B) << endl;

    double val = M_PI/2;
    for(int i=0; i < 6; ++i) {
        q6.setZero();
        q6(i) = val;
        akin.armFK(B, qz+q6, SIDE_RIGHT);
        printf("q(%d) = %f, B = \n", i, val);
        cout << zero_it(B) << endl;
    }
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
