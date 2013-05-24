#include "test_utils.h"
#include <amino.h>

using namespace std;
using namespace Eigen;
using namespace atlas;
using namespace robot;

using namespace kinematics;
using namespace dynamics;
using namespace simulation;
/* ********************************************************************************************* */
void PRINT_KINEMATIC_CHAIN(BodyNode *end_effector) {
    printf("%s kinematic chain:\n", end_effector->getName());
    Joint *parent = end_effector->getParentJoint();
    while(parent) {
        printf("%s\n", parent->getName());
        if(!parent->getParentNode())
            break;
        parent = parent->getParentNode()->getParentJoint();
    }
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_INIT) {
    PREPARE_ROBOT_JACOBIAN();
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_DART) {
    // Examine DART conventions for returning Jacobians
    Skeleton *robot = PREPARE_ROBOT();
    PRINT_KINEMATIC_CHAIN(robot->getNode(ROBOT_LEFT_HAND));
    BodyNode *end_effector = robot->getNode(ROBOT_LEFT_HAND);
    printf("dependent dofs:\n");
    for(int i=0; i < end_effector->getNumDependentDofs(); i++) {
        Dof *dof = robot->getDof(end_effector->getDependentDof(i));
        printf("%d %s %s\n", i, dof->getJoint()->getName(), dof->getName());
    }
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_REMAP_JACOBIAN) {
    robot_jacobian_t* rjac = PREPARE_ROBOT_JACOBIAN();
    
    MatrixXd J(3, 6);
    for(int i=0; i < J.cols(); i++)
        J.col(i) = i*Vector3d::Ones();
    
    vector<int> dependent = { 0, 1, 2, 3, 4, 5 };
    vector<int> desired = { 0, 5, 1 };
    
    rjac->remap_jacobian(J, dependent, desired);

    cout << "remapped J = \n" << J << endl;

}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_MANIP_JACOBIAN) {
    Skeleton *robot = PREPARE_ROBOT();
    robot_jacobian_t* rjac = PREPARE_ROBOT_JACOBIAN();
    robot_state_t* rstat = PREPARE_ROBOT_STATE();
    
    MatrixXd J;
    vector<int> desired_dofs;
    rstat->get_manip_indexes(desired_dofs, LIMB_L_ARM);
    BodyNode *l_arm = robot->getNode(ROBOT_LEFT_HAND);
    VectorXd dofs = robot->getPose();
    dofs.setZero();

    rstat->dofs() = dofs;

    rjac->manip_jacobian(J, desired_dofs, l_arm, *rstat);

    VectorXd qdot(J.cols());
    VectorXd x(6);
    x << 0, -0.01, 0, 0, 0, 0;
    
    aa_la_dls(J.rows(), J.cols(), 0.1, J.data(), x.data(), qdot.data());

    rstat->dart_pose() = dofs;
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_ANGLE_AXIS) {
    Isometry3d B;
    Isometry3d W;
    W = Matrix4d::Identity();
    B = Matrix4d::Identity();
    
    W.rotate(AngleAxisd(-M_PI/2, Vector3d::UnitX()));
    B.rotate(AngleAxisd(M_PI/2, Vector3d::UnitY()));

    Matrix3d R = B.rotation();

    AngleAxisd ob;
    ob.fromRotationMatrix(B.rotation());
    AngleAxisd ow;
    ow.fromRotationMatrix(W.rotation());

    cout << "ob = \n" << ob.axis() << endl;
    cout << "angle = \n" << ob.angle() << endl;

    cout << "ow = \n" << ow.axis() << endl;
    cout << "angle = " << ow.angle() << endl;

    AngleAxisd wb;
    wb.fromRotationMatrix(  (W.inverse() * B).rotation() );
   
    cout << "wb = \n" << wb.axis() << endl;
    cout << "angle = " << wb.angle() << endl;
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_MANIP_IK) {
    robot_jacobian_t* robot = PREPARE_ROBOT_JACOBIAN();
    robot_state_t* state = PREPARE_ROBOT_STATE();

    vector<int> desired_dofs;
    state->get_manip_indexes(desired_dofs, LIMB_L_ARM);
    
    state->print_joints(desired_dofs);

    BodyNode *left_hand = state->robot()->getNode(ROBOT_LEFT_HAND);
    state->dofs().setZero();

    VectorXd q(desired_dofs.size());

    for(int i=0; i < desired_dofs.size(); i++) {
        q(i) = 0.1 * i;
    }
    // q << 0, 0, 0.5, 0, 0, 0;o
    state->set_manip(q, LIMB_L_ARM);

    state->robot()->setPose(state->dofs());
    Isometry3d B;
    B = left_hand->getWorldTransform();
    cout << "B = \n" << B.matrix() << endl;

    state->dofs().setZero();
    
    robot->manip_jacobian_ik(B, desired_dofs, left_hand, *state);

    state->print_joints(desired_dofs);
    
    cout << "ans = \n" << left_hand->getWorldTransform() << endl;
    
}
/* ********************************************************************************************* */
TEST(JACOBIAN, TEST_MANIP_IK_BASE_FRAME) {
    robot_jacobian_t *robot = PREPARE_ROBOT_JACOBIAN();
    robot_state_t *state = PREPARE_ROBOT_STATE();

    state->dofs().setZero();
    state->copy_into_robot();

    MatrixXd J;
    vector<int> desired;
    BodyNode *end_effector;

    state->get_manip_indexes(desired, LIMB_L_ARM);
    end_effector = state->robot()->getNode(ROBOT_LEFT_HAND);

    robot->manip_jacobian(J, desired, end_effector, *state);

    MatrixXd K;
    Isometry3d Twb;
    Twb = Matrix4d::Identity();

    Twb.rotate(AngleAxisd(M_PI/2, Vector3d::UnitZ()));
    state->set_body(Twb);

    robot->manip_jacobian(K, desired, end_effector, *state);

    // cout << "J=\n" << J << endl;
    // cout << "K=\n" << K << endl;

    // ASSERT_MATRIX_EQ(J, K); //< meant to fail

    BodyNode *head = state->robot()->getNode(ROBOT_HEAD);
    Isometry3d Twh;
    Twh = head->getWorldTransform();

    Isometry3d Twf;
    BodyNode *l_foot = state->robot()->getNode(ROBOT_LEFT_FOOT);
    Twf = l_foot->getWorldTransform();

    state->get_manip_indexes(desired, LIMB_L_LEG);

    Twh(1,3) -= 0.1;

    robot->manip_jacobian_ik(Twh, desired, l_foot, head, *state);

    ASSERT_MATRIX_EQ(Twh.matrix(), head->getWorldTransform(), 1e-5);
    ASSERT_MATRIX_EQ(Twf.matrix(), l_foot->getWorldTransform(), 1e-5);
    
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
