#include "test_utils.h"

using namespace std;
using namespace Eigen;

using namespace kinematics;
using namespace simulation;
using namespace dynamics;

using namespace robot;
using namespace atlas;
/* ********************************************************************************************* */
TEST(STATE, TEST_INIT) {
    robot_state_t *rs = PREPARE_ROBOT_STATE();

    rs->init(PREPARE_ROBOT());
    robot_state_t::print_mappings();

    vector<int> full_body;
    rs->get_full_indexes(full_body);
    rs->print_nodes(full_body);
}
/* ********************************************************************************************* */
TEST(STATE, TEST_BODY) {
    robot_state_t * rs = PREPARE_ROBOT_STATE();
    rs->init(PREPARE_ROBOT());
    Skeleton *robot = PREPARE_ROBOT();

    Isometry3d Twb;
    Twb = Matrix4d::Identity();

    VectorXd dofs = robot->getPose();
    dofs.setZero();
    dofs(0) = 10;
    dofs(1) = 200;
    dofs(2) = -12012;
    dofs(3) = 1;
    dofs(4) = M_PI/2;
    dofs(5) = 2;
    robot->setPose(dofs);

    Twb = robot->getNode(ROBOT_BODY)->getWorldTransform();

    rs->set_body(Twb);
    Matrix4d Tnb;
    rs->get_body(Tnb);

    ASSERT_MATRIX_EQ(Twb.matrix(), Tnb);

    // cout << "Twb\n" << Twb.matrix() << endl;
    // cout << "Tnb\n" << Tnb << endl;
}
/* ********************************************************************************************* */
TEST(STATE, TEST_CLAMP) {
    robot_state_t *rs = PREPARE_ROBOT_STATE();
    VectorXd q;
    rs->get_manip(q, LIMB_L_ARM);
    vector<int> indexes;
    rs->get_manip_indexes(indexes, LIMB_L_ARM);
    for(int i=0; i < indexes.size(); i++) {
        q(i) = rs->robot()->getDof(indexes[i])->getMax();
    }
    rs->set_manip(q, LIMB_L_ARM);
    ASSERT_TRUE(rs->clamp_indexes(indexes));
    for(int i=0; i < indexes.size(); i++) {
        q(i) = 100;
    }
    rs->set_manip(q, LIMB_L_ARM);
    ASSERT_FALSE(rs->clamp_indexes(indexes));
}
/* ********************************************************************************************* */
TEST(STATE, TEST_CHAIN_INDEXES) {
    Skeleton *robot = PREPARE_ROBOT();
    robot_state_t *state = PREPARE_ROBOT_STATE();
    
    vector<int> chain;
    state->get_chain_indexes(chain, robot->getNode(ROBOT_LEFT_HAND), robot->getNode(ROBOT_RIGHT_HAND));
//    state->print_children(chain);
    
    state->get_chain_indexes(chain, robot->getNode(ROBOT_LEFT_HAND), robot->getNode(ROBOT_RIGHT_FOOT));
//    state->print_children(chain);

    vector<int> full_body;
    state->get_full_indexes(full_body);
    for(int i=0; i < full_body.size(); i++) {
        state->print_backchain(full_body[i]);
        cout << endl;
        state->print_dependent_dofs(full_body[i]);
        cout << endl << endl;
    }
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
