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

void poseCallback( const geometry_msgs::PosePtr &_pose ) {

  xi = _pose->position.x;
  yi = _pose->position.y;
  zi = _pose->position.z;
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

  // Give time
  ros::Duration(1.0).sleep();

  // Spin
  ros::spinOnce();

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

  zd.generateZMPGait( max_steps );

  // Convert it to a message
  pjt_msg = zd.getPoseJointTrajMsg( xi,
				    yi,
				    zi);
  // Give it a time
  pjt_msg.header.stamp = ros::Time::now();


  // Send it
  printf("Publishing pose animation \n");
  poseJointTrajPub.publish( pjt_msg );

  printf("SPIN NOW \n");

  // Spin
  ros::spinOnce();
  
  // Wait
  ros::Duration(30).sleep();

	// Send it again
  // Give it a time
  pjt_msg.header.stamp = ros::Time::now();
  printf("Publishing pose animation again! \n");
  poseJointTrajPub.publish( pjt_msg );

  // Spin
  ros::spinOnce();
  
  // Wait
  ros::Duration(10).sleep();



}
