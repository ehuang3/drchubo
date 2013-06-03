/**
 * @file main.cpp
 * @brief Send continuous data to robot joints (no trajectories)
 * @author A. Huaman
 * @date 2013/06/01
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Dense>

const int gNumBodyDofs = 24; // Left Arm, Right Arm, Left Leg, Right Leg
const int gNumJoints = 35;
std::string gJointNames[] = {"drchubo::LSP", "drchubo::LSR", "drchubo::LSY", "drchubo::LEP", "drchubo::LWY", "drchubo::LWP", "drchubo::LWR",
			     "drchubo::RSP", "drchubo::RSR", "drchubo::RSY", "drchubo::REP", "drchubo::RWY", "drchubo::RWP", "drchubo::RWR",
			     "drchubo::LHY", "drchubo::LHR", "drchubo::LHP", "drchubo::LKP", "drchubo::LAP", "drchubo::LAR",
			     "drchubo::RHY", "drchubo::RHR", "drchubo::RHP", "drchubo::RKP", "drchubo::RAP", "drchubo::RAR",
			     "drchubo::TSY", "drchubo::NKY", "drchubo::NKP",
			     "drchubo::LF1", "drchubo::LF2", "drchubo::LF3",
			     "drchubo::RF1", "drchubo::RF2", "drchubo::RF3"};


int main( int argc, char* argv[] ) {

  ros::init( argc, argv, "move_arm" );

  ros::NodeHandle* node = new ros::NodeHandle();
  
  ros::Time last_ros_time_;
  // Wait until sim is active (play)
  bool wait = true;
  
  while( wait ) {
    last_ros_time_ = ros::Time::now();
    if( last_ros_time_.toSec() > 0 ) {
      wait = false;
    }
  }

  ros::Publisher modePub = node->advertise<std_msgs::String>( "drchubo/mode", 1, false );
  // For grab and release
  ros::Publisher grabPub = node->advertise<geometry_msgs::Pose>( "drc_world/robot_grab_link", 1, false );
  // And for robot configuration 
  ros::Publisher releasePub = node->advertise<geometry_msgs::Pose>( "drc_world/robot_release_link",
								     1, false );

  // Give time
  ros::Duration(0.2).sleep();

  // Start by setting mode to no_gravity
  std_msgs::String mode_msg;
  mode_msg.data = "stay_dog";

  modePub.publish( mode_msg );
  ros::spinOnce();
  ros::Duration(0.1).sleep();


  // Send grasp
  printf( "Sent grab \n" );
  geometry_msgs::Pose drillPose;

  drillPose.position.x = 0;
  drillPose.position.y = 0;
  drillPose.position.z = -.4;

  Eigen::Isometry3d Tf;
  Tf = Eigen::Matrix4d::Identity();
  Tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()));
  Tf.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
      
  Eigen::Quaterniond quat;
  quat = Tf.linear();

  drillPose.orientation.w = quat.w();
  drillPose.orientation.x = quat.x();
  drillPose.orientation.y = quat.y();
  drillPose.orientation.z = quat.z();

  releasePub.publish( drillPose );

  ros::Duration(0.2).sleep();

  grabPub.publish( drillPose );
  

  ros::spinOnce();

  // Give time
  ros::Duration(0.2).sleep();
  

  return 0;
}
