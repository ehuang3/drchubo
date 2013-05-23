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
    atlas_state_t as;
    as.init(PREPARE_ROBOT());
    atlas_state_t::print_mappings();
}
/* ********************************************************************************************* */
TEST(STATE, TEST_BODY) {
    atlas_state_t as;
    as.init(PREPARE_ROBOT());
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

    Twb = robot->getNode("pelvis")->getWorldTransform();

    as.set_body(Twb);
    Matrix4d Tnb;
    as.get_body(Tnb);

    ASSERT_MATRIX_EQ(Twb.matrix(), Tnb);

    // cout << "Twb\n" << Twb.matrix() << endl;
    // cout << "Tnb\n" << Tnb << endl;
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
