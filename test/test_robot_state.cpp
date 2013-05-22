#include <iostream>
#include <gtest/gtest.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Transformation.h>
#include <dynamics/SkeletonDynamics.h>

#include <utils/data_paths.h>
#include <robot/robot_state.h>
#include <atlas/atlas_state.h>

using namespace std;
using namespace Eigen;

using namespace kinematics;
using namespace simulation;
using namespace dynamics;

using namespace robot;
using namespace atlas;
/* ********************************************************************************************* */
#define ROBOT_URDF "models/atlas/atlas_world.urdf"
#define ROBOT_NAME "atlas"
kinematics::Skeleton* _robot_;
Skeleton* PREPARE_ROBOT() {
    if(_robot_ == 0) {
        DartLoader dart_loader;
		World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH ROBOT_URDF);
		_robot_ = mWorld->getSkeleton(ROBOT_NAME);
    }
    return _robot_;
}
/* ********************************************************************************************* */
TEST(STATE, TEST_INIT) {
    atlas_state_t as;
    as.init(PREPARE_ROBOT());
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
