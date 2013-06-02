/**
 * @file main.cpp
 * @brief Send continuous data to robot joints (no trajectories)
 * @author A. Huaman
 * @date 2013/06/01
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <DRC_msgs/PoseJointTrajectory.h>


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

  // For mode
  ros::Publisher modePub = node->advertise<std_msgs::String>( "drchubo/mode", 1, false );
  ros::Publisher animPub = node->advertise<DRC_msgs::PoseJointTrajectory>( "drchubo/poseJointAnimation", 
										    1,
										    false );

  // Give time
  ros::Duration(1.0).sleep();

  // Start by setting mode to no_gravity
  std_msgs::String mode_msg;
  mode_msg.data = "no_gravity";

  modePub.publish( mode_msg );
  ros::spinOnce();
  ros::Duration(0.2).sleep();
  

  // Send a message every time for some time
  double dt = 0.01;
  double T = 5.0;
  int numSteps = (int)( T/dt );
  
  printf("Start loop of commands \n");
  double t = 0;
  // Do it
  for( int i = 0; i < numSteps; ++i ) {

    // Set the default animation message
    DRC_msgs::PoseJointTrajectory pjt;
    pjt.header.stamp = ros::Time::now();
    pjt.header.frame_id = "drchubo::Body_Torso";
    
    for( int i = 0; i < gNumJoints; ++i ) {
      pjt.joint_names.push_back( gJointNames[i] );  
    }
    
    // Only one point
    trajectory_msgs::JointTrajectoryPoint jt;
    geometry_msgs::Pose p;

    double angle = -1.57 / (double) numSteps;
    double val = angle*i;
    

    jt.positions.resize( gNumJoints );
    // Generate the message
    for( int j = 0; j < gNumJoints; ++j ) {
      jt.positions[j] = 0.0;
      // If left shoulder pitch
      if( strcmp( pjt.joint_names[j].c_str(), "drchubo::LSP" ) == 0 ) {
	jt.positions[j] = val;
      } 
      else if( strcmp( pjt.joint_names[j].c_str(), "drchubo::RSP" ) == 0 ) {
	jt.positions[j] = val;
      } 
    }

    // Set poses
    p.position.x = 0.0;
    p.position.y = 0.0;
    p.position.z = 1.5;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = 0;
    p.orientation.w = 1.0;

    
    pjt.points.push_back(jt);
    pjt.poses.push_back(p);


    pjt.points[0].time_from_start = ros::Duration().fromSec(dt);

    // Publish it
    animPub.publish( pjt );
    // Spin
    ros::spinOnce();
    // Sleep for a little while so we don't send stuff too quickly
    ros::Duration(dt).sleep();
  
  }
  
  printf("Start a little nap before closing this node \n");
  ros::Duration(1.0).sleep();
  printf("Done \n");

  // Give time
  ros::Duration(0.2).sleep();
  

  return 0;
}
