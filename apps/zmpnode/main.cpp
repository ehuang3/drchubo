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

  // Store the last joint configuration to set the robot at the end of the animation 
  //(to avoid him from flying and hitting himself)
  sensor_msgs::JointState lastState;
  trajectory_msgs::JointTrajectoryPoint lastPoint;
  int numPoints = pjt_msg.points.size();
  lastPoint = pjt_msg.points[numPoints - 1];

  for( int i = 0; i < pjt_msg.joint_names.size(); ++i ) {
    lastState.name.push_back( pjt_msg.joint_names[i] );
    lastState.position.push_back( lastPoint.positions[i] );
  }


  // Send it
  printf("Publishing pose animation \n");
  poseJointTrajPub.publish( pjt_msg );

  printf("SPIN NOW \n");

  // Spin
  ros::spinOnce();
  
  // Wait
  ros::Duration(10).sleep();
  /*
  printf("Send stay dog \n");
  // Send stayDog mode
  // First send the configuration you want stay dog to stay in

  confPub.publish( lastState );
  ros::spinOnce();
  ros::Duration(0.5).sleep(); 

  std_msgs::String mode_msg;
  mode_msg.data = "stay_dog";
  modePub.publish( mode_msg );
  ros::Duration(1.0).sleep();
  printf("End \n");
  */
}
