/**
 * @file trajTester.h
 */
#ifndef _TRAJ_TESTER_GOLEM_
#define _TRAJ_TESTER_GOLEM_


#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Core>

/** 
 * @class trajTester
 */

class trajTester {

 public:
  trajTester();
  void init();
  void demo_1();
  void jointTrajCommand( std::vector<Eigen::VectorXd> _rawTraj,
			 double _dt );

 private:
  trajectory_msgs::JointTrajectory mJt;
  ros::NodeHandle* mNode;
  ros::Publisher mTrajPub;
};

#endif /** _TRAJ_TESTER_GOLEM_ */
