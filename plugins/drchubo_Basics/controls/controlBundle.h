/**
 * @file controlBundle.h
 */

#ifndef __CONTROL_BUNDLE_HUBO__
#define __CONTROL_BUNDLE_HUBO__

#include <iostream>
#include <vector>
#include <gazebo/physics/Joint.hh>
#include "pid_controller.h"

/**
 * @class controlBundle
 */
class controlBundle {

 public:
  controlBundle();
  ~controlBundle();
  void setSize( int _num );
  void setJoints( std::vector<physics::JointPtr> _joints );
  void setTargets( std::vector<double> _targets );
  void initPID( int _index, double _Kp, double _Ki, double _Kd, 
		double _Imax, double _Imin, 
		double _outputMax, double _outputMin );
  void updateControls( double _dt );
  std::vector<pid_controller> mPIDs;
  std::vector<double> mTargets;
  std::vector<physics::JointPtr> mJoints;
  int mNumActuatedJoints;
};

#endif /** __CONTROL_BUNDLE_HUBO__ */
