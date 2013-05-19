/**
 * @file Climber.h
 */

#ifndef _CLIMBER_VRC_GOLEM_H
#define _CLIMBER_VRC_GOLEM_H

#include <ros/ros.h>
#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasBehaviorStepData.h>
#include <sensor_msgs/Imu.h>


class Climber {
 public:
  void init();
  void demo();

  // Callbacks
  void state_cb( const atlas_msgs::AtlasSimInterfaceState& _asis_msg );
  void imu_cb( const sensor_msgs::Imu& _imu_msg );

 private:
  int mNumJoints;

  ros::NodeHandle* mNode;

  /**< Latest message from /atlas/imu */
  sensor_msgs::Imu mImu_msg;
  /**< Latest message from /atlas/atlas_sim_interface_state */
  atlas_msgs::AtlasSimInterfaceState mAsis_msg;


  ros::Publisher mAc_pub;
  ros::Publisher mAsic_pub;

  ros::Subscriber mAsis_sub;
  ros::Subscriber mImu_sub;
  
  double mFrequency;
  ros::Rate* mLoopRate;
};

#endif /* _CLIMBER_VRC_GOLEM_H */
