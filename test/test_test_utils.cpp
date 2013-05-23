#include "test_utils.h"


/* ********************************************************************************************* */
TEST(TEST_UTILS, PREPARE_ROBOT) {
    kinematics::Skeleton* robot = PREPARE_ROBOT();
    ASSERT_TRUE(robot);
}
/* ********************************************************************************************* */
TEST(TEST_UTILS, PREPARE_JACOBIAN) {
    robot::robot_jacobian_t* robot = PREPARE_ROBOT_JACOBIAN();
    ASSERT_TRUE(robot);
}
/* ********************************************************************************************* */
TEST(TEST_UTILS, PREPARE_ROBOT_STATE) {
    robot::robot_state_t* robot = PREPARE_ROBOT_STATE();
    ASSERT_TRUE(robot);
}
/* ********************************************************************************************* */
TEST(TEST_UTILS, PREPARE_ROBOT_KINEMATICS) {
    robot::robot_kinematics_t* robot = PREPARE_ROBOT_KINEMATICS();
    ASSERT_TRUE(robot);
}
/* ********************************************************************************************* */
TEST(TEST_UTILS, ASSERT_MATRIX_EQUALS) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d B = Eigen::Matrix4d::Identity();
    ASSERT_MATRIX_EQ(A, B);
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
