#include "test_utils.h"

using namespace std;
using namespace Eigen;

using namespace kinematics;
using namespace simulation;
using namespace dynamics;

using namespace robot;
using namespace hubo;

// Global variables for use in functions below
kinematics::Skeleton* robot_skel;  // DART skeleton of hubo
hubo::hubo_state_t* hubo_state;    // Tracks DART joint angles (dofs) and allows conversion into ROS joint angles plus easy getters and setters
hubo::hubo_kinematics_t* hubo_kin; // Kinematics solver for DRC-HUBO (See test functions below for usage)
hubo::hubo_jacobian_t* hubo_jac;   // Jacobians for DRC-HUBO
/* ********************************************************************************************* */
TEST(HUBO, SETUP)
{
    // This function demonstrates how to set up and initialize the hubo/ classes.

    // Use DART to load in a Skeleton drc-hubo
    DartLoader dart_loader; // This loads the world file
    // The macros VRC_DATA_PATH and ROBOT_URDF are defined in utils/data_paths.h and utils/robot_configs.h, respectively
    simulation::World *mWorld = dart_loader.parseWorld(DRC_DATA_PATH ROBOT_URDF);
    // Likewise with the ROBOT_NAME macro
    dynamics::SkeletonDynamics* hubo_skel = mWorld->getSkeleton(ROBOT_NAME);

    // Initialize hubo classes
    hubo::hubo_state_t* state = new hubo::hubo_state_t();
    hubo::hubo_kinematics_t* kin = new hubo::hubo_kinematics_t();
    hubo::hubo_jacobian_t* jac = new hubo::hubo_jacobian_t();

    // Warning: These classes now share and use the HUBO skeleton loaded above for computations
    state->init(hubo_skel);
    kin->init(hubo_skel);
    jac->init(hubo_skel);

    // Write to globals for the guides below
    robot_skel = hubo_skel;
    hubo_state = state;
    hubo_kin = kin;
    hubo_jac = jac;
}
/* ********************************************************************************************* */
TEST(HUBO, STATE)
{
    // This function demonstrates how to use the hubo_state_t class

    // hubo_state_t inherits from robot_state_t.
    // robot_state_t implements all the functionality, hubo_state_t just defines
    // the mapping.

    // Functions take in a reference to the state,
    // compute, and write the result into that state.
    hubo::hubo_state_t &state = *hubo_state;

    // robot_state_t stores the joint angles in the same format that 
    // DART's kinematics::Skeleton::getPose() returns.

    // robot_state_t has a VectorXd called _dofs that stores those joint
    // angles

    // 1. Retrieve the joint angles from hubo_state_t 
    Eigen::VectorXd dofs = state.dart_pose();

    // 2. Zero Hubo
    dofs.setZero();
    state.set_dart_pose(dofs); // Writes the state back into _dofs

    // To use DART's kinematics functions we need to write the
    // dofs of a robot_state_t into the kinematics::Skeleton

    kinematics::Skeleton* hubo_skel = state.robot(); // Retrieve the Skeleton*
    hubo_skel->setPose(state.dart_pose());

    // 3. Obtain the Tf of Hubo's left wrist

    // ROBOT_LEFT_HAND is a macro for the name of drchubo's left wrist link
    // See "utils/robot_configs.h" for additional macros (like left foot or torso)

    Eigen::Isometry3d left_wrist(hubo_skel->getNode(ROBOT_LEFT_HAND)->getWorldTransform());

    // Retrive and set the left arm joint angles
    // See robot_state.h for all the variations on this

    // robot::MANIP_L_HAND is defined in robot/robot.h
    Eigen::VectorXd left_arm;
    state.get_manip(left_arm, robot::MANIP_L_HAND);

    // Retrive the indexes of the left arm joints in the DART dof array
    std::vector<int> left_arm_indexes;
    state.get_manip_indexes(left_arm_indexes, robot::MANIP_L_HAND);
    state.get_manip_indexes(left_arm_indexes, robot::LIMB_L_ARM);

    // This clamps the left arm angles
    left_arm << 100, 21043, 121421, 12931, 1023, 591;
    state.set_manip(left_arm, robot::MANIP_L_HAND);

    state.check_limits(left_arm_indexes);
    state.clamp_manip(robot::MANIP_L_HAND, false);

    // 

}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */