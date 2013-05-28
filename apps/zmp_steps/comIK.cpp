/**
 * @file comIK.cpp
 */
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/BodyNode.h>
#include <utils/data_paths.h>

#include <atlas/atlas_kinematics.h>
#include <robot/robot_kinematics.h>

kinematics::Skeleton *_atlas;

/**
 * @function prepareAtlasKinematics
 */
atlas::atlas_kinematics_t *prepareAtlasKinematics() {

  atlas::atlas_kinematics_t *_ak;
  DartLoader dart_loader;
  simulation::World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH "models/atlas/atlas_world.urdf");
  _atlas = mWorld->getSkeleton("atlas");
  _ak = new atlas::atlas_kinematics_t();
  _ak->init(_atlas);
       
  _atlas->setPose(_atlas->getPose().setZero(), true);
  return _ak;
}

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  atlas::atlas_kinematics_t *AK = prepareAtlasKinematics();
    
  Eigen::Matrix4d Twb = _atlas->getNode("pelvis")->getWorldTransform();
  //cout << "Twb=\n" << Twb << endl;
  Eigen::Matrix4d Twl = AK->legFK( Eigen::Vector6d::Zero(), true);
  Eigen::Matrix4d Twr = AK->legFK( Eigen::Vector6d::Zero(), false);

  Eigen::Matrix4d Tm[robot::robot_kinematics_t::NUM_MANIPULATORS];
  Tm[robot::robot_kinematics_t::MANIP_L_FOOT] = Twl;
  Tm[robot::robot_kinematics_t::MANIP_R_FOOT] = Twr;

  robot::robot_kinematics_t::IK_Mode mode[robot::robot_kinematics_t::NUM_MANIPULATORS];
  mode[robot::robot_kinematics_t::MANIP_L_FOOT] = robot::robot_kinematics_t::IK_MODE_SUPPORT;
  mode[robot::robot_kinematics_t::MANIP_R_FOOT] = robot::robot_kinematics_t::IK_MODE_WORLD;
  mode[robot::robot_kinematics_t::MANIP_L_HAND] = robot::robot_kinematics_t::IK_MODE_FIXED;
  mode[robot::robot_kinematics_t::MANIP_R_HAND] = robot::robot_kinematics_t::IK_MODE_FIXED;

  Eigen::VectorXd dofs = _atlas->getPose();
  dofs.setZero();
  _atlas->setPose(dofs, true);

  Eigen::Vector3d com = _atlas->getWorldCOM();

  std::cout << "\n current com = \n" << _atlas->getWorldCOM().transpose() << std::endl;

  com(2) -= .15;

  std::cout << "\n desired com = \n" << com.transpose() << std::endl;

  // Apply IK
  AK->comIK( _atlas, com, Twb, mode, Tm, dofs );

  Eigen::Vector4d ncom = Eigen::Vector4d::Ones();
  ncom.block<3,1>(0,0) = _atlas->getWorldCOM();

  //ncom = (Twb * ncom);

  std::cout << "\n new com = \n" << ncom.block<3,1>(0,0).transpose() << std::endl;
  
  _atlas->setPose(dofs, true);
  
  std::cout << "\n dofd com = \n" << _atlas->getWorldCOM().transpose() << std::endl;
  
  std::cout << "Error : " <<(com - ncom.block<3,1>(0,0)).norm() << std::endl;

  /*
  /// Test Dart Skeleton
  dofs.setZero();
  _atlas->setPose(dofs, true);
  cout << "zero com=\n" << _atlas->getWorldCOM().transpose() << endl;
  
  dofs.block(0,0,3,1) = Vector3d(0, 0, 0.15);
  _atlas->setPose(dofs, true);
  cout << "0.15 com=\n" << _atlas->getWorldCOM().transpose() << endl;
  
  dofs.setZero();
  _atlas->setPose(dofs, true);
  cout << "pelvis=\n" << _atlas->getNode("pelvis")->getWorldTransform() << endl;
  
  dofs.block(0,0,3,1) = Vector3d(0, 0, 0.15);
  _atlas->setPose(dofs, true);
  cout << "pelvis=\n" << _atlas->getNode("pelvis")->getWorldTransform() << endl;
  
  // Assert that all joints have moved up by 0.15 m
  int nNodes = _atlas->getNumNodes();
  dofs.setZero();
  _atlas->setPose(dofs, true);
  EIGEN_V_MAT4D x0;
  for(int i=0; i < nNodes; i++) {
    x0.push_back(_atlas->getNode(i)->getWorldTransform());
  }
  
  dofs.block(0,0,3,1) = Vector3d(0, 0, 0.15);
  _atlas->setPose(dofs, true);
  EIGEN_V_MAT4D x1;
  for(int i=0; i < nNodes; i++) {
    x1.push_back(_atlas->getNode(i)->getWorldTransform());
  }
  
  double XFORM_TOL = 1e-10;
  for(int i=0; i < nNodes; i++) {
    
    //cout << _atlas->getNode(i)->getName() << endl;
    
    ASSERT_NEAR(x0[i](2,3)+0.15, x1[i](2,3), XFORM_TOL);
    
    for(int r=0; r < 4; r++)
      for(int c=0; c < 4; c++)
	if(!(r==2 && c==3))
	  ASSERT_NEAR(x0[i](r,c), x1[i](r,c), XFORM_TOL);
  }
  
  // Assert that all the com contributions have moved up by 0.15
  dofs.setZero();
  _atlas->setPose(dofs, true);
  EIGEN_V_VEC3D c0;
  for(int i=0; i < nNodes; i++) {
    BodyNode *node = _atlas->getNode(i);
    c0.push_back(node->getWorldCOM());
  }
  
  dofs.block(0,0,3,1) = Vector3d(0, 0, 0.15);
  _atlas->setPose(dofs, true);
  EIGEN_V_VEC3D c1;
  for(int i=0; i < nNodes; i++) {
    BodyNode *node = _atlas->getNode(i);
    c1.push_back(node->getWorldCOM());
  }
  
  double mass = 0;
  for(int i=0; i < nNodes; i++) {
    BodyNode *node = _atlas->getNode(i);
    //		cout << node->getName() << endl;
    //
    //		cout << "c0=\n"<< c0[i].transpose() << endl;
    //		cout << "c1=\n"<< c1[i].transpose() << endl;
    //
    //		cout << "c0*m=\n" << (node->getMass() * c0[i]).transpose() << endl;
    //		cout << "c1*m=\n" << (node->getMass() * c1[i]).transpose() << endl;
    
    //ASSERT_NEAR(c0[i](2), c1[i](2), XFORM_TOL);
    
    mass += node->getMass();
  }
  
  Vector3d calc_com(0,0,0);
  for(int i=0; i < nNodes; i++) {
    BodyNode *node = _atlas->getNode(i);
    calc_com += (node->getMass() * node->getWorldCOM());
  }
  calc_com = calc_com / _atlas->getMass();
  cout << "calc_com=\n" << calc_com.transpose() << endl;
  
  cout << "mass = " << mass << endl;
  cout << "mass = " << _atlas->getMass() << endl;
  */
}
