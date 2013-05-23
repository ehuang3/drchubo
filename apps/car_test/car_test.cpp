/**
 * @file car_test.cpp
 */
#include <ros/ros.h>
#include <std_msgs/Float64.h>


/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  ros::init( argc, argv, "car_test" );
  
  ros::NodeHandle* node;
  ros::Publisher gasPub;
  ros::Publisher handBrakePub;

  node = new ros::NodeHandle();

  // Wait until it is active
    ros::Time last_ros_time_;
    bool wait = true;
    while (wait)
    {
      last_ros_time_ = ros::Time::now();
      if (last_ros_time_.toSec() > 0)
	wait = false;
    }
    
	// Set publisher for hand brake (set by default, so if you don't disengage it, the car won't move)
  handBrakePub = node->advertise<std_msgs::Float64>("/drc_vehicle/hand_brake/cmd", 0, true );
  std_msgs::Float64 handBrake_msg;
  handBrake_msg.data = 0.0;

  // Send
 	handBrakePub.publish( handBrake_msg );

  // Give it time to act
  ros::spinOnce();
  ros::Duration(3.0).sleep();
  printf("Brake applied, next is pedal message \n");

	
  // Set publisher for gas pedal
  gasPub = node->advertise<std_msgs::Float64>("/drc_vehicle/gas_pedal/cmd", 1, true );

  // Set message to half speed (0 - 1)
  std_msgs::Float64 gas_msg;
  gas_msg.data = 0.9;

  // Send
  gasPub.publish( gas_msg );
  printf("Gas pedal message just sent  \n");
  // Give it time to act
  ros::spinOnce();
  ros::Duration(3.0).sleep();
  printf("Gas pedal  \n");

  ros::spin();

  return 0;
}

