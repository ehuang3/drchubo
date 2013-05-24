/**
 * @file pid_controller.cpp
 */

#include <math.h>
#include <stdio.h>
#include <gazebo/math/Helpers.hh>
#include "pid_controller.h"


/**
 * @function pid_controller
 * @brief
 */
pid_controller::pid_controller( double _Kp, double _Ki, double _Kd,
				 double _Imax, double _Imin,
				 double _outputMax, double _outputMin ) 
  : Kp(_Kp), Ki(_Ki), Kd(_Kd), Imax(_Imax), Imin(_Imin), outputMax(_outputMax), outputMin(_outputMin) {
  this->reset();
}

/**
 * @function pid_controller
 * @brief
 */
pid_controller::~pid_controller() {

}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::init( double _Kp, double _Ki, double _Kd,
			    double _Imax, double _Imin,
			    double _outputMax, double _outputMin ) {

  this->Kp = _Kp;
  this->Ki = _Ki;
  this->Kd = _Kd;
  this->Imax = _Imax;
  this->Imin = _Imin;
  this->outputMax = _outputMax;
  this->outputMin = _outputMin;

  this->reset();
}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::setPGain( double _Kp ) {
  Kp = _Kp;
}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::setIGain( double _Ki ) {
  Ki = _Ki;
}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::setDGain( double _Kd ) {
  Kd = _Kd;
}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::setIMax( double _Imax ) {
  Imax = _Imax;
}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::setIMin( double _Imin ) {
  Imin = _Imin;
}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::setOutputMax( double _outputMax ) {
  outputMax = _outputMax;
}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::setOutputMin( double _outputMin ) {
  outputMin = _outputMin;
}

/**
 * @function pid_controller
 * @brief
 */
double pid_controller::update( double _Ep, common::Time _dt ) {
 
  double Xp; double Xd; double Xi;

  this->Ep = _Ep;
  
  if( _dt == common::Time(0,0) || math::isnan(_Ep) || isinf(_Ep) ) {
    return 0.0;
  }

  // Calculate P value added to the control signal
  Xp = this->Kp*this->Ep;

  // Calculate I error
  this->Ei = this->Ei + _dt.Double()*this->Ep;

  // Calculate I value added to the control signal
  Xi = this->Ki*this->Ei;

  // Limit Xi so that the limit is meaningful in the control signal
  if( Xi > this->Imax ) {
    Xi = this->Imax;
    this->Ei = Xi / this->Ki;
  }
  else if( Xi < this->Imin ) {
    Xi = this->Imin;
    this->Ei = Xi / this->Ki;
  }

  // Calculate D error
  if( _dt != common::Time(0,0) ) {
    this->Ed = ( this->Ep - this->Ep_prev ) / _dt.Double();
    this->Ep_prev = this->Ep;
  }

  // Calculate D value added to the control signal
  Xd = this->Kd*this->Ed;

  // Add them all together
  this->output = -Xp - Xi - Xd;

  // Check command limits
  if( !math::equal( this->outputMax, 0.0 ) && this->output > this->outputMax ) {
    this->output = this->outputMax;
  }
  if( !math::equal( this->outputMin, 0.0 ) && this->output > this->outputMin ) {
    this->output = this->outputMin;
  }
 
  return this->output;
}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::setOutput( double _output ) {
  this->output = _output;
}

/**
 * @function pid_controller
 * @brief
 */
double pid_controller::getOutput() {
  return this->output;
}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::getErrors( double &_Ep, double &_Ei, double &_Ed ) {
  _Ep = this->Ep;
  _Ei = this->Ei;
  _Ed = this->Ed;
}

/**
 * @function pid_controller
 * @brief
 */
void pid_controller::reset() {

  this->Ep_prev = 0.0;
  this->Ep = 0.0;
  this->Ei = 0.0;
  this->Ed = 0.0;
  this->output = 0.0;
  this->outputMax = 0.0;
  this->outputMin = 0.0;
}

