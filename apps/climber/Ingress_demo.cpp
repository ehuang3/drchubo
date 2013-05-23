/**
 * @file demo.cpp
 */
#include <ros/ros.h>
#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasBehaviorStepData.h>
#include <sensor_msgs/Imu.h>

#include "Ingress.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  ros::init( argc, argv, "control_mode_switch" );
  
  Ingress in;
  in.init();
  in.looking_ahead();
  in.grab_handle();
  //in.demo();
}

