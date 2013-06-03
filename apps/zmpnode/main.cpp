/**
 * @file main.cpp
 */
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <DRC_msgs/PoseStampedArray.h>
#include <DRC_msgs/PoseJointTrajectory.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "zmpnode.h"

double xi, yi, zi, qxi, qyi, qzi, qwi;
sensor_msgs::JointStatePtr initJointState;
geometry_msgs::PosePtr initPose;

/**
 * @function poseCallback
 */
void poseCallback( const geometry_msgs::PosePtr &_pose ) {

  initPose = _pose;
} 

/**
 * @function jointCallback
 */
void jointCallback( const sensor_msgs::JointStatePtr &_jointState ) {  
  initJointState = _jointState;
}

int main( int argc, char* argv[] ) {

  ros::init( argc, argv, "zmp_walk_gait" );

  ros::NodeHandle* node = new ros::NodeHandle();
  
  // Set a publisher for trajectory (joints AND body pose)
  ros::Publisher poseJointTrajPub = node->advertise<DRC_msgs::PoseJointTrajectory>( "drchubo/poseJointAnimation", 
										    1,
										    false );
  // For mode
  ros::Publisher modePub = node->advertise<std_msgs::String>( "drchubo/mode", 1, false );
  // And for robot configuration
  ros::Publisher confPub = node->advertise<sensor_msgs::JointState>( "drchubo/configuration",
								     1, false );

  // For current robot pose
  ros::Subscriber poseSub = node->subscribe( "drchubo/pose", 1, poseCallback );
  // For current joint state
  ros::Subscriber jointStateSub = node->subscribe( "drchubo/jointStates", 1, jointCallback );

  // Give time
  ros::Duration(1.0).sleep();

  // Spin
  ros::spinOnce();
  ros::Duration(0.1).sleep();

  // Get the gait
  zmpnode zd;
  trajectory_msgs::JointTrajectory traj_msg;
  DRC_msgs::PoseStampedArray pose_msg;  
  DRC_msgs::PoseJointTrajectory pjt_msg;

  // Generate ZMP trajectories
  // Default variables
  size_t max_steps;

  // Get params from server (if no default)
  int walk_max_steps;
  if( node->getParam("/walk_max_steps", walk_max_steps ) ){
     max_steps = walk_max_steps;
  } else { printf("No /walk_max_steps parameter set. SET IT NOW OR I WON'T WALK! \n"); }	

   max_steps = 5;
  zd.generateZMPGait( max_steps );
  ros::spinOnce();
  ros::Duration(0.1).sleep();

  // Send it
  double transitionTime = 1.0;
  for( int i = 0; i < 4; ++i ) {
  // Convert it to a message
    pjt_msg = zd.getPoseJointTrajMsg( initPose, initJointState, transitionTime );
    printf("Publishing pose animation: [%d] Time \n", i );
    pjt_msg.header.stamp = ros::Time::now();
    poseJointTrajPub.publish( pjt_msg );
  
  // Wait
  ros::Duration(7).sleep();

  // Spin
  printf("Spin \n");
  ros::spinOnce();
 
  }

  printf("We are done - Getting out of node \n");


}
