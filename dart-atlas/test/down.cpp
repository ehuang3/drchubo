#include <iostream>
#include <Eigen/Dense>
//#include <gtest/gtest.h>

#include <atlas/AtlasKinematics.h>
#include <atlas/AtlasUtils.h>
#include <utils/AtlasPaths.h>


#include <robotics/parser/dart_parser/DartLoader.h>
#include <robotics/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <dynamics/SkeletonDynamics.h>

#include <fstream>
#include <string>

using namespace std;
using namespace Eigen;
using namespace atlas;

using namespace kinematics;
using namespace dynamics;
using namespace robotics;


atlas::AtlasKinematics *_ak;
kinematics::Skeleton *_atlas;

atlas::AtlasKinematics *prepareAtlasKinematics() {
	if(!_ak) {
		DartLoader dart_loader;
		World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH "atlas/atlas_world.urdf");
		_atlas = mWorld->getSkeleton("atlas");
		_ak = new AtlasKinematics();
		_ak->init(_atlas);
	}
	_atlas->setPose(_atlas->getPose().setZero(), true);
	return _ak;
}

int main()
{
	ofstream myfile;
	myfile.open("init.yaml");

	AtlasKinematics *AK = prepareAtlasKinematics();

	cout << endl;
	int nDofsNum = _atlas->getNumDofs();
	VectorXd dofs(nDofsNum);

	dofs(6) = 0;
	dofs(9) = 0.0017;
	dofs(12) = 0; 
	dofs(16) = 0.781;

	dofs(7) = -0.0021;
	dofs(10) = 0.0623;
	dofs(13) = -0.2654;
	dofs(18) = 0.4837; 
	dofs(23) = -0.2012; 
	dofs(27) = -0.0623;

	dofs(8) = 0.0021;
	dofs(11) = -0.0623; 
	dofs(14) = -0.2654; 
	dofs(19) = 0.4835;
	dofs(24) = -0.20122;
	dofs(28) = 0.0623,

	dofs(15) = 0.2978;
	dofs(20) = -1.3140;
	dofs(25) = 2.0021;
	dofs(29) = 0.4955;
	dofs(31) = 0; 
	dofs(33) = -0.01;

	dofs(17) = 0.2978;
	dofs(22) = 1.3140;
	dofs(26) = 2.0021;
	dofs(30) = -0.4955; 
	dofs(32) = 0;
	dofs(34) = 0.01;

	VectorXd aindex(28);
	aindex(0) = 6;
	aindex(1) = 9;
	aindex(2) = 12;
	aindex(3) = 16;

	aindex(4) = 7;
	aindex(5) = 10;
	aindex(6) = 13;
	aindex(7) = 18;
	aindex(8) = 23;
	aindex(9) = 27;

	aindex(10) = 8;
	aindex(11) = 11;
	aindex(12) = 14;
	aindex(13) = 19;
	aindex(14) = 24;
	aindex(15) = 28;

	aindex(16) = 15;
	aindex(17) = 20;
	aindex(18) = 25;
	aindex(19) = 29;
	aindex(20) = 31;
	aindex(21) = 33;

	aindex(22) = 17;
	aindex(23) = 22;
	aindex(24) = 26;
	aindex(25) = 30;
	aindex(26) = 32;
	aindex(27) = 34;


//	cout << "before set: " << dofs << endl;

	for (int j = 1; j <= 5000; j++) { 

		/**************************/
		myfile << "  - [0.001, \"";
		for (int i = 0; i < 28; i++) {
			myfile << dofs(aindex(i)) / 5000 * j << ' ';
		}
		myfile << "\"]\n";
	}

	myfile.close();

//	check << dLeftx << "\n";
//	_atlas->setPose(dofs, true, false);
//	dofs = _atlas->getPose();
//	cout << "after set: " << dofs << endl;

//	Vector3d com = _atlas->getWorldCOM();
//	cout << "com: " << com << endl;

//	cout << "pelvis: \n" << _atlas->getNode("pelvis")->getWorldTransform() << endl;
//	cout << "left foot: \n" << _atlas->getNode("l_foot")->getWorldTransform() << endl;
//	cout << "right foot: \n" << _atlas->getNode("r_foot")->getWorldTransform() << endl;
}
