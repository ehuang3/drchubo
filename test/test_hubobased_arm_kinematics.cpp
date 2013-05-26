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
robot_arm_constants_t PREPARE_CONSTANTS()
{
    robot_arm_constants_t rc;
    rc.arm_nsy = 0;
    rc.arm_ssz = 0; //< atlas needs this
    rc.arm_sez = 2; //< shoulder to elbow
    rc.arm_ewz = 1; // elbow to wrist
    rc.arm_whz = 0.1; //< wrist to end effector
    
    rc.arm_offset(0) = 0;
    rc.arm_offset(1) = 0;
    rc.arm_offset(2) = 0;
    rc.arm_offset(3) = 0;
    rc.arm_offset(4) = 0;
    rc.arm_offset(5) = 0;

    rc.arm_limits << 
        -1, 2,
        -1, 2,
        -1, 2,
        -1, 2,
        -1, 2,
        -1, 2;
    
    rc.arm_mirror.push_back(1);
    rc.arm_mirror.push_back(2);
    rc.arm_mirror.push_back(4);

    return rc;
}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, TEST_INIT) {
    robot_arm_kinematics_t ak;
    robot_arm_constants_t rc = PREPARE_CONSTANTS();
    ak.set_constants(rc);
    
    Isometry3d B;
    Vector6d q;
    int side;
}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, TEST_FK) {
    robot_arm_kinematics_t ak;
    robot_arm_constants_t rc = PREPARE_CONSTANTS();
    ak.set_constants(rc);

    Isometry3d B;
    Vector6d q;
    int side;

    q.setZero();
    side = SIDE_LEFT;

    ak.armFK(B, q, side);
}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, TEST_IK) {
    robot_arm_kinematics_t ak;
    robot_arm_constants_t rc = PREPARE_CONSTANTS();
    ak.set_constants(rc);

    Isometry3d B;
    Vector6d q;
    int side;

    q.setZero();
    side = SIDE_LEFT;

    q << 1, 1, 1, 1, 1, 1;

    ak.armFK(B, q, side);

    Vector6d qans;
    ak.armIK(qans, B, qans, side);

    ASSERT_MATRIX_EQ(q, qans);
}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, TEST_JOINT_DIRECTIONS) {
    robot_arm_kinematics_t ak;
    robot_arm_constants_t rc = PREPARE_CONSTANTS();
    ak.set_constants(rc);

    Isometry3d B;
    Vector6d q;
    int side;

    q.setZero();
    side = SIDE_LEFT;

    ak.armFK(B, q, side);
    printf("q = zero, B = \n");
    cout << zero_it(B) << endl;

    double val = M_PI/2;
    for(int i=0; i < q.size(); i++) {
        q.setZero();
        q(i) = val;
        ak.armFK(B, q, side);
        
        printf("q(%d) = %f, B =\n", i, val);
        cout << zero_it(B) << endl;
    }
}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, TEST_DART_DIRECTIONS) {
    robot_arm_kinematics_t ak;
    robot_arm_constants_t rc = PREPARE_CONSTANTS();
    ak.set_constants(rc);

    robot_state_t* state = PREPARE_ROBOT_STATE();
    state->dofs().setZero();
    state->copy_into_robot();

    //
    PREPARE_ROBOT_KINEMATICS();


    vector<int> arm_indexes;
    state->get_manip_indexes(arm_indexes, MANIP_L_HAND);
    VectorXd q(6);
    BodyNode *left_hand = state->robot()->getNode(ROBOT_LEFT_HAND);
    BodyNode *left_shoulder = state->robot()->getNode("l_clav");
    
    Isometry3d Tws;
    Isometry3d B;
    
    Tws = left_shoulder->getWorldTransform();
    B = left_hand->getWorldTransform();

    printf("q is zero, B = \n");
    cout << zero_it(B) << endl;

    double val = M_PI/2;
    for(int i=0; i < q.size(); i++) {
        q.setZero();
        q(i) = val;
        
        state->set_dofs(q, arm_indexes);
        state->copy_into_robot();
        
        B = left_hand->getWorldTransform();
        B = Tws.inverse() * B;

        printf("q(%d) = %f, B = \n", i, q(i));
        cout << zero_it(B) << endl;
    }

}
/* ********************************************************************************************* */
TEST(ARM_KINEMATICS, TEST_JOINT_OFFSETS) {
    robot_arm_kinematics_t ak;
    robot_arm_constants_t rc = PREPARE_CONSTANTS();
    ak.set_constants(rc);

    Isometry3d B;
    Vector6d q;
    int side;

    q.setZero();
    side = SIDE_LEFT;

    ak.armFK(B, q, side);
    printf("q = zero, B = \n");
    cout << zero_it(B) << endl;

    double val = M_PI/2;
    for(int i=0; i < q.size(); i++) {
        rc.arm_offset.setZero();
        rc.arm_offset(i) = val;
        ak.set_constants(rc);

        ak.armFK(B, q, side);
        
        printf("offset(%d) = %f, B =\n", i, val);
        cout << zero_it(B) << endl;
    }
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
