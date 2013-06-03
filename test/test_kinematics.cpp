#include "test_utils.h"

using namespace std;
using namespace Eigen;

using namespace kinematics;
using namespace simulation;
using namespace dynamics;

using namespace robot;
using namespace atlas;
/* ********************************************************************************************* */
TEST(KINEMATICS, TEST_INIT)
{
    robot_state_t *rs = PREPARE_ROBOT_STATE();
    rs->init(PREPARE_ROBOT());
    PREPARE_ROBOT_KINEMATICS();
}
/* ********************************************************************************************* */
TEST(KINEMATICS, TEST_LEG_WORLD_TO_DH)
{
    robot_kinematics_t *rk = PREPARE_ROBOT_KINEMATICS();
    robot_state_t *rs = PREPARE_ROBOT_STATE();
    Skeleton *robot= rs->robot();

    rs->dofs().setZero();
    
    
    VectorXd q(6);
    q << 0, 1, 2, 3, 4, 5;

    rs->set_manip(q, LIMB_L_LEG);

    robot->setPose(rs->dart_pose());
    Isometry3d B;
    Isometry3d Bdh;
    B = rs->robot()->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();

    rs->get_manip(q, LIMB_L_LEG);
    Bdh = rk->legFK(q, true);

    ASSERT_MATRIX_EQ(rk->leg_world_to_dh(B.matrix()), Bdh.matrix());
}
/* ********************************************************************************************* */
TEST(KINEMATICS, TEST_LEG_IK)
{
    robot_kinematics_t *rk = PREPARE_ROBOT_KINEMATICS();
    robot_state_t *rs = PREPARE_ROBOT_STATE();
    Skeleton *robot = rs->robot();
    
    VectorXd q(6);
    q << 0, -1, 2, -3, 4, -5;

    rs->set_manip(q, LIMB_L_LEG);
    rs->clamp_manip(MANIP_L_FOOT, false);
    robot->setPose(rs->dart_pose());

    Isometry3d Twb;
    Twb.rotate(AngleAxisd(1, Vector3d::UnitX()));
    Twb.translate(Vector3d(0,1,3));
    Twb.rotate(AngleAxisd(-1, Vector3d::UnitZ()));
    rs->set_body(Twb);
    robot->setPose(rs->dart_pose());
    
    Isometry3d B;
    B = rs->robot()->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
    
    rs->dofs().setZero();
    rs->set_body(Twb);
    robot->setPose(rs->dart_pose());

    rk->leg_ik(B, true, *rs);

    robot->setPose(rs->dart_pose());

    ASSERT_MATRIX_EQ(B.matrix(), rs->robot()->getNode(ROBOT_LEFT_FOOT)->getWorldTransform());

    
}
/* ********************************************************************************************* */
TEST(KINEMATICS, TEST_STANCE_IK)
{
    Isometry3d end_effectors[NUM_MANIPULATORS];
    IK_Mode mode[NUM_MANIPULATORS];

    robot_kinematics_t* rk = PREPARE_ROBOT_KINEMATICS();
    robot_state_t* rs = PREPARE_ROBOT_STATE();
    Skeleton* robot = rs->robot();

    BodyNode* left_foot = rs->robot()->getNode(ROBOT_LEFT_FOOT);
    BodyNode* right_foot = rs->robot()->getNode(ROBOT_RIGHT_FOOT);

    rs->dofs().setZero();
    robot->setPose(rs->dart_pose());

    end_effectors[MANIP_L_FOOT] = rs->robot()->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
    end_effectors[MANIP_R_FOOT] = rs->robot()->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();
    
    mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
    mode[MANIP_R_FOOT] = IK_MODE_WORLD;
    mode[MANIP_L_HAND] = IK_MODE_FIXED;
    mode[MANIP_R_HAND] = IK_MODE_FIXED;

    // Test stance IK

    Isometry3d Twb;
    rs->get_body(Twb);
    // move body down
    Twb.translation() = Twb.translation() + Vector3d(0, 0, -0.1);
    // Twb.rotate(AngleAxisd(M_PI, Vector3d::UnitY()));
    // write it in
    rs->set_body(Twb);

    rk->stance_ik(end_effectors, mode, *rs);

    // verify
    ASSERT_MATRIX_EQ(end_effectors[MANIP_L_FOOT].matrix(), left_foot->getWorldTransform());
    ASSERT_MATRIX_EQ(end_effectors[MANIP_R_FOOT].matrix(), right_foot->getWorldTransform());
}
/* ********************************************************************************************* */
TEST(KINEMATICS, TEST_COM_IK)
{
    Vector3d world_com;
    Isometry3d end_effectors[NUM_MANIPULATORS];
    IK_Mode mode[NUM_MANIPULATORS];

    robot_kinematics_t* rk = PREPARE_ROBOT_KINEMATICS();
    robot_state_t* rs = PREPARE_ROBOT_STATE();

    BodyNode* left_foot = rs->robot()->getNode(ROBOT_LEFT_FOOT);
    BodyNode* right_foot = rs->robot()->getNode(ROBOT_RIGHT_FOOT);
    Skeleton* robot = rs->robot();

    rs->dofs().setZero();
    robot->setPose(rs->dart_pose());

    end_effectors[MANIP_L_FOOT] = rs->robot()->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
    end_effectors[MANIP_R_FOOT] = rs->robot()->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();

    mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
    mode[MANIP_R_FOOT] = IK_MODE_WORLD;
    mode[MANIP_L_HAND] = IK_MODE_FIXED;
    mode[MANIP_R_HAND] = IK_MODE_FIXED;

    world_com = rs->robot()->getWorldCOM();
    world_com(2) -= 0.1;

    rk->com_ik(world_com, end_effectors, mode, *rs);

    cout << "L FOOT before =\n" << end_effectors[MANIP_L_FOOT].matrix() << endl;
    cout << "L FOOT after =\n" << left_foot->getWorldTransform() << endl;
    
    ASSERT_MATRIX_EQ(end_effectors[MANIP_L_FOOT].matrix(), rs->robot()->getNode(ROBOT_LEFT_FOOT)->getWorldTransform());
    ASSERT_MATRIX_EQ(end_effectors[MANIP_R_FOOT].matrix(), rs->robot()->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform());    
    ASSERT_MATRIX_EQ(world_com, rs->robot()->getWorldCOM(), 1e-3);

}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
