/**
 * @file zmpnode.h
 */
#ifndef _ZMP_DEMO_H_
#define _ZMP_DEMO_H_

#include "zmp/hubo-zmp.h"
#include <math.h>
#include "mzcommon/MzGlutApp.h"
#include "mzcommon/TimeUtil.h"
#include <getopt.h>

#include "zmp/zmpwalkgenerator.h"

// ROS Msg 
#include <trajectory_msgs/JointTrajectory.h>
#include <DRC_msgs/PoseStampedArray.h>
#include <DRC_msgs/PoseJointTrajectory.h>

// Print stuff
#include <stdio.h>
#include <map>


using namespace fakerave;

typedef std::vector< zmp_traj_element_t > TrajVector;
/*
size_t seconds_to_ticks(double s) {
  return size_t(round(s*TRAJ_FREQ_HZ));
}*/

const int stance_foot_table[4] = { 0, 1, 0, 1 };
const int swing_foot_table[4] = { -1, -1, 1, 0 };

const stance_t next_stance_table[4] = {
  SINGLE_LEFT,
  SINGLE_RIGHT,
  DOUBLE_RIGHT,
  DOUBLE_LEFT
};

enum walktype {
  walk_canned,
  walk_line,
  walk_circle
};


/**
 * @class zmpnode
 */
class zmpnode {
  
 public:
  zmpnode();
  ~zmpnode();    
  
  void generateZMPGait(); 
  
  trajectory_msgs::JointTrajectory getJointTrajMsg();
  DRC_msgs::PoseStampedArray getPoseTrajMsg();
  DRC_msgs::PoseJointTrajectory getPoseJointTrajMsg();
  // Helpers
  Eigen::Matrix4d tf2Mx( Transform3 _tf );
  
  
 private:
  
  // Variables for matt's parsing types
  double mdt;
  static const int mNumBodyDofs;
  static const int mNumJoints;
  static std::string mJointNames[];
  std::vector< Eigen::VectorXd > mMzJointTraj;
  std::vector<double> mRootX;
  std::vector<double> mRootY;
  std::vector<double> mRootZ;
  
};


#endif /** _ZMP_DEMO_H_  */ 
