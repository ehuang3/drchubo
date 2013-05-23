/**
 * @file trajTester.cpp
 * @brief 
 */
#include "trajTester.h"

#include <atlas/atlas_kinematics.h>
#include <utils/data_paths.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>


/**
 * @function trajTester
 */
trajTester::trajTester() {

}

/**
 * @function init
 */
void trajTester::init() {

  // Initialize the node
  mNode = new ros::NodeHandle( "trajTester" );
  
  ros::Duration(1.0).sleep();

  // Set publisher
  mTrajPub = mNode->advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1, true ); 

  // Set joint Trajectory command  
  // Dummy set time at init
  mJt.header.stamp = ros::Time::now();
  mJt.header.frame_id = "atlas::pelvis";
  // Set joint names
  mJt.joint_names.push_back( "atlas::back_lbz" );
  mJt.joint_names.push_back( "atlas::back_mby" );
  mJt.joint_names.push_back( "atlas::back_ubx" );
  mJt.joint_names.push_back( "atlas::neck_ay"  );
  mJt.joint_names.push_back( "atlas::l_leg_uhz" );
  mJt.joint_names.push_back( "atlas::l_leg_mhx" );
  mJt.joint_names.push_back( "atlas::l_leg_lhy" );
  mJt.joint_names.push_back( "atlas::l_leg_kny" );
  mJt.joint_names.push_back( "atlas::l_leg_uay" );
  mJt.joint_names.push_back( "atlas::l_leg_lax" ); 
  mJt.joint_names.push_back( "atlas::r_leg_lax" );
  mJt.joint_names.push_back( "atlas::r_leg_uay" );
  mJt.joint_names.push_back( "atlas::r_leg_kny" );
  mJt.joint_names.push_back( "atlas::r_leg_lhy" );
  mJt.joint_names.push_back( "atlas::r_leg_mhx" );
  mJt.joint_names.push_back( "atlas::r_leg_uhz" );
  mJt.joint_names.push_back( "atlas::l_arm_elx" );
  mJt.joint_names.push_back( "atlas::l_arm_ely" );
  mJt.joint_names.push_back( "atlas::l_arm_mwx" );
  mJt.joint_names.push_back( "atlas::l_arm_shx" );
  mJt.joint_names.push_back( "atlas::l_arm_usy" );
  mJt.joint_names.push_back( "atlas::l_arm_uwy" );
  mJt.joint_names.push_back( "atlas::r_arm_elx" );
  mJt.joint_names.push_back( "atlas::r_arm_ely" );
  mJt.joint_names.push_back( "atlas::r_arm_mwx" );
  mJt.joint_names.push_back( "atlas::r_arm_shx" );
  mJt.joint_names.push_back( "atlas::r_arm_usy" );
  mJt.joint_names.push_back( "atlas::r_arm_uwy" );
}

/**
 * @function demo_1
 */
void trajTester::demo_1() {

  init();

  
  std::vector<Eigen::VectorXd> traj;

  // Let generate a down movement (lowering the CoM)

  // Init kinematic class
  DartLoader dl;
  simulation::World* world = dl.parseWorld( VRC_DATA_PATH "models/atlas/atlas_world.urdf" );
  kinematics::Skeleton* atlasSkel = world->getSkeleton( "atlas" );
  
  atlas::atlas_kinematics_t* ak = new atlas::atlas_kinematics_t();
  ak->init( atlasSkel );
  
  atlasSkel->setPose( atlasSkel->getPose().setZero(), true );
  
  // CoM IK
  Eigen::Matrix4d Twb = atlasSkel->getNode("pelvis")->getWorldTransform();
  Eigen::Matrix4d Twl = ak->legFK(Eigen::Vector6d::Zero(), true );
  Eigen::Matrix4d Twr = ak->legFK(Eigen::Vector6d::Zero(), false );


  // 

  int n = 100;
  int num = 28;
  double dt = 0.01;
  double theta;
  double x1, x2;
  
  Eigen::VectorXd p = Eigen::VectorXd::Zero( num );
  for( int i = 0; i < n; ++i ) { 
	/*
      theta = 2*3.1416*0.05*i*dt;
      x1 = -0.5*sin(2*theta);
      x2 =  0.5*sin(1*theta);
      p[0] = x1; p[1] = x2; p[2] = x2; p[3] = x2;
      p[4] = x2; p[5] = x2; p[6] = x1; p[7] = x2; p[8] = x1; p[9] = x1;
      p[10] = x2; p[11] = x1; p[12] = x2; p[13] = x1; p[14] = x1; p[15] = x2;
      p[16] = x2; p[17] = x1; p[18] = x1; p[19] = x1; p[20] = x2; p[21] = x2;
      p[22] = x2; p[23] = x1; p[24] = x1; p[25] = x2; p[26] = x1; p[27] = x1;
*/
      double factor = (double) i / (double) n ;
      p[7] = 1.0*factor;	
      traj.push_back( p );
  }

  jointTrajCommand( traj, 0.05 );
}

/**
 * @function jointTrajCommand
 */
void trajTester::jointTrajCommand( std::vector<Eigen::VectorXd> _rawTraj,
				   double _dt ) {

  // Enter the data
  for( int i = 0; i < _rawTraj.size(); ++i ) {
    
    // Create trajectory point from data
    trajectory_msgs::JointTrajectoryPoint p;
    for( int j = 0; j < _rawTraj[i].size(); ++j ) {
      p.positions.push_back( _rawTraj[i](j) );
    }
    // Add to the trajectory
    mJt.points.push_back( p );

    // Set time stamp accordingly
    mJt.points[i].time_from_start = ros::Duration( _dt );
  }

  mTrajPub.publish( mJt );
  
}
