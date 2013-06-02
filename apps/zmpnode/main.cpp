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

  // Give time
  ros::Duration(1.0).sleep();

  // Get the gait
  zmpnode zd;
  trajectory_msgs::JointTrajectory traj_msg;
  DRC_msgs::PoseStampedArray pose_msg;  
  DRC_msgs::PoseJointTrajectory pjt_msg;

  // Generate ZMP trajectories
  zd.generateZMPGait();

  // Convert it to a message
  pjt_msg = zd.getPoseJointTrajMsg();
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
