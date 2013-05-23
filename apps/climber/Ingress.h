/**
 * @file Ingress.h
 */

#ifndef _Ingress_VRC_GOLEM_H
#define _Ingress_VRC_GOLEM_H

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


class Ingress {
 public:
  void init();
  void demo();
  /**< Hard-coded walk to locate Atlas in front of car */
  void ingress_1();

  void getToPose( std::vector<double> _goalPose,
		  double _time = 4.0,
		  int _frequency = 100 ); 
  void switchToBdiStandMode();

  // Callback: Store IMU, state, force-torque and a few more things
  void as_cb( const atlas_msgs::AtlasState& _as_msg );
  // Other callback
  void state_cb( const atlas_msgs::AtlasSimInterfaceState& _asis_msg );

  // Constants used often
  const static float sKp_pos[];
  const static double sInit_pos[];

 private:

  int mNumJoints;

  ros::NodeHandle* mNode;

  /**< Latest message with Atlas state (IMU, force torque, joints) */
  atlas_msgs::AtlasState mAs_msg;
  /**< Latest message from /atlas/atlas_sim_interface_state */
  atlas_msgs::AtlasSimInterfaceState mAsis_msg;


  ros::Publisher mAc_pub;
  ros::Publisher mAsic_pub;

  ros::Subscriber mAsis_sub;
  ros::Subscriber mImu_sub;
  ros::Subscriber mAs_sub;
};

#endif /* _Ingress_VRC_GOLEM_H */
