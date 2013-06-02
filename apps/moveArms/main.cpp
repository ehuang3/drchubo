/**
 * @file main.cpp
 * @brief Send continuous data to robot joints (no trajectories)
 * @author A. Huaman
 * @date 2013/06/01
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>


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
  // And for robot configuration 
  ros::Publisher confPub = node->advertise<sensor_msgs::JointState>( "drchubo/configuration",
								     10, false );

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

  sensor_msgs::JointState conf_msg;

  // Set defaults
  for( int i = 0; i < gNumJoints; ++i ) {
    conf_msg.name.push_back( gJointNames[i] );
    conf_msg.position.push_back( 0.0 );
  }

  // Set stay dog
/*
  mode_msg.data = "stay_dog";

  modePub.publish( mode_msg );
  ros::spinOnce();
  ros::Duration(0.2).sleep();
  */
  printf("Start loop of commands \n");
  
  // Do it
  for( int i = 0; i < numSteps; ++i ) {

    conf_msg.header.stamp = ros::Time::now();
    printf(" i: %d \n", i );
    double angle = -1.57 / (double) numSteps;
    double val = angle*i;

    // Generate the message
    for( int j = 0; j < gNumJoints; ++j ) {
      conf_msg.position[j] = 0.0;
      // If left shoulder pitch
      if( strcmp( conf_msg.name[j].c_str(), "drchubo::LSP" ) == 0 ) {
	conf_msg.position[j] = val;
      } 
      else if( strcmp( conf_msg.name[j].c_str(), "drchubo::RSP" ) == 0 ) {
	conf_msg.position[j] = val;
      } 
    }

    // Publish it
    confPub.publish( conf_msg );
    // Spin
    ros::spinOnce();
    // Sleep for a little while so we don't send stuff too quickly
    //ros::Duration(dt).sleep();
  
  }
  
  printf("Start a little nap before closing this node \n");
  ros::Duration(1.0).sleep();
  printf("Done \n");

  // Give time
  ros::Duration(0.2).sleep();
  

  return 0;
}
