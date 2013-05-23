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
TEST(STATE, TEST_D_BODY) {
    atlas_state_t as;
    as.init(PREPARE_ROBOT());
    Skeleton *robot = PREPARE_ROBOT();

    Isometry3d Twb;
    Twb = Matrix4d::Identity();

    VectorXd dofs = robot->getPose();
    dofs.setZero();
    dofs(3) = 1;
    dofs(4) = M_PI/2;
    dofs(5) = 2;
    robot->setPose(dofs);

    Twb = robot->getNode("pelvis")->getWorldTransform();

    as.set_d_body(Twb);
    robot->setPose(as.d_pose());

    ASSERT_MATRIX_EQ(Twb.matrix(), robot->getNode("pelvis")->getWorldTransform());
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
