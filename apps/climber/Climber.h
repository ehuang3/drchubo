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

#include <actionlib/client/simple_action_client.h>
#include <atlas_msgs/WalkDemoAction.h>

#include <vector>


class Climber {
 public:
  void init();
  void demo();
  /**< Hard-coded walk to locate Atlas in front of car */
  void blindWalk();
  void blindStraightWalk( double _dist,
			  double _stepLength,
			  double _stepDist = 0.15 );

  std::vector<atlas_msgs::AtlasBehaviorStepData> takeNSteps( int _numSteps,
							     double _stepLength,
							     double _stepDist = 0.15 );

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

  actionlib::SimpleActionClient<atlas_msgs::WalkDemoAction>* mClient;
  
  double mFrequency;
  ros::Rate* mLoopRate;
};

#endif /* _CLIMBER_VRC_GOLEM_H */
