#include <iostream>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <atlas/AtlasKinematics.h>
#include <atlas/AtlasUtils.h>
#include <utils/AtlasPaths.h>


#include <robotics/parser/dart_parser/DartLoader.h>
#include <robotics/World.h>
#include <kinematics/Skeleton.h>
#include <dynamics/SkeletonDynamics.h>

using namespace std;
using namespace Eigen;
using namespace atlas;

using namespace kinematics;
using namespace dynamics;
using namespace robotics;

/* ********************************************************************************************* */
TEST(KINEMATICS, INIT) {
	DartLoader dart_loader;
	World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH "atlas/atlas_world.urdf");
	SkeletonDynamics *atlas = mWorld->getSkeleton("atlas");

	AtlasKinematics AK;
	AK.init(atlas);
}
/* ********************************************************************************************* */
TEST(KINEMATICS, FORWARD) {
	DartLoader dart_loader;
	World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH "atlas/atlas_world.urdf");
	SkeletonDynamics *atlas = mWorld->getSkeleton("atlas");

	AtlasKinematics AK;
	AK.init(atlas);

	Matrix4d Tfoot = AK.legFK(0,0,0,0,0,0);

	cout << Tfoot << endl;
}
/* ********************************************************************************************* */
TEST(KINEMATICS, INVERSE) {
	AtlasKinematics AK;
	AK.init(0);
	const double TOLERANCE_EXACT = 1.0e-10;

	const int NUM_SLICE = 3;
	double a[NUM_SLICE];
	for(int i=0; i < NUM_SLICE; ++i) {
		a[i] = -M_PI + 1 + double(i)/NUM_SLICE*(2*M_PI - 1);
	}
	VectorXd u(6);
	Matrix4d Tfoot;
	for(int i1=0; i1 < NUM_SLICE; ++i1)
	for(int i2=0; i2 < NUM_SLICE; ++i2)
	for(int i3=0; i3 < NUM_SLICE; ++i3)
	for(int i4=0; i4 < NUM_SLICE; ++i4)
	for(int i5=0; i5 < NUM_SLICE; ++i5)
	for(int i6=0; i6 < NUM_SLICE; ++i6) {
		Tfoot = AK.legFK(a[i1], a[i2], a[i3], a[i4], a[i5], a[i6]);
		u(0) = a[i1]; u(1) = a[i2]; u(2) = a[i3]; u(3) = a[i4]; u(4) = a[i5]; u(5) = a[i6];

		MatrixXd v = AK.legIK(Tfoot);

		cout << "u=\n" << u << endl;
		cout << "v=\n" << v << endl;

		bool match = false;
		for(int i=0; i < 8; ++i) {
			VectorXd err = (v.block(0,i,6,1) - u);
			if(err.norm() < TOLERANCE_EXACT) {
				match = true;
			}
		}
		if(!match)
			ASSERT_EQ(0, 1);
	}
}
/* ********************************************************************************************* */
TEST(KINEMATICS, INVERSE_NEAREST) {
	DartLoader dart_loader;
	World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH "atlas/atlas_world.urdf");
	SkeletonDynamics *atlas = mWorld->getSkeleton("atlas");

	AtlasKinematics AK;
	AK.init(atlas);

	Vector6d u, a;
	u << 0, 0, 0, 0, 0, 1;

	Matrix4d Tfoot = AK.legFK(u);

	a = AK.legIK(Tfoot, u);
	cout << a << endl;

	cout << "ans= \n" << AK.legFK(a) << endl;
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
