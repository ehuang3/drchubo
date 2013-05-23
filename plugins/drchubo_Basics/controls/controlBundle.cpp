/**
 * @file controlBundle.cpp
 */
#include "controlBundle.h"


/** 
 * @function controlBundle
 * @brief
 */
controlBundle::controlBundle() {

}

/** 
 * @function ~controlBundle
 * @brief
 */
controlBundle::~controlBundle() {

}

/** 
 * @function setSize
 * @brief
 */
void controlBundle::setSize( int _num ) {

  mNumActuatedJoints = _num;

  mPIDs.resize( mNumActuatedJoints );
  mTargets.resize( mNumActuatedJoints );
  mJoints.resize( mNumActuatedJoints );
}

/** 
 * @function setJoints
 * @brief
 */ 
void controlBundle::setJoints( std::vector<physics::JointPtr> _joints ) {
  mJoints = _joints;
}

/** 
 * @function setTargets
 * @brief
 */
void controlBundle::setTargets( std::vector<double> _targets ) {

  if( _targets.size() != mNumActuatedJoints ) {
    std::cout << "Num targets is not the same as number of actuated joints. No setting targets!" << std::endl;
    return;
  }

  mTargets = _targets;

  for( int i = 0; i < _targets.size(); ++i ) {
    mPIDs[i].setOutput( mTargets[i] );
  }
}

/** 
 * @function initPID
 * @brief
 */
void controlBundle::initPID( int _index, double _Kp, double _Ki, double _Kd, 
			     double _Imax, double _Imin, 
			     double _outputMax, double _outputMin  ) {

  mPIDs[_index].init( _Kp, _Ki, _Kd,
		      _Imax, _Imin, 
		      _outputMax, _outputMin );
}

/** 
 * @function updateControls
 * @brief
 */
void controlBundle::updateControls( double _dt ) {
  
  double error;

  for( int i = 0; i < mNumActuatedJoints; ++i ) {
    error = mJoints[i]->GetAngle(0).Radian() - mTargets[i];
    mPIDs[i].update( error, _dt );
    mJoints[i]->SetForce( 0, mPIDs[i].getOutput() );
  }

}

