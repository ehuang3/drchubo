/**
 * @file pid_controller.h
 */

#ifndef __PID_CONTROLLER_HUBOLEG_H__
#define __PID_CONTROLLER_HUBOLEG_H__


#include <gazebo/common/Time.hh>

using namespace gazebo;

/**
 * @class pid_controller
 */
class pid_controller {
 public:

  pid_controller( double _Kp = 0.0, double _Ki = 0.0, double _Kd = 0.0,
		  double _Imax = 0.0, double _Imin = 0.0,
		  double _outputMax = 0.0, double _outputMin = 0.0 );
  ~pid_controller();

  void init( double _Kp = 0.0, double _Ki = 0.0, double _Kd = 0.0,
	     double _Imax = 0.0, double _Imin = 0.0,
	     double _outputMax = 0.0, double _outputMin = 0.0 );
  void setPGain( double _Kp );
  void setIGain( double _Ki );
  void setDGain( double _Kd );

  void setIMax( double _Imax );
  void setIMin( double _Imin );
  void setOutputMax( double _outputMax );
  void setOutputMin( double _outputMin );

  double update( double _error, common::Time _dt );
  void setOutput( double _output );
  double getOutput();

  void getErrors( double &_Ep, double &_Ei, double &_Ed );

  pid_controller &operator=(const pid_controller &_pid ) {
    if( this == &_pid ) {
      return *this;
    }
    
    this->Kp = _pid.Kp;
    this->Ki = _pid.Ki;
    this->Kd = _pid.Kd;

    this->Imax = _pid.Imax;
    this->Imin = _pid.Imin;
    this->outputMax = _pid.outputMax;

    this->reset();
    return *this;    
  } // end operator

  void reset();
  double Ep;
  double Ep_prev;
  double Ei;
  double Ed;
  double Kp;
  double Ki;
  double Kd;
  
  double Imax;
  double Imin;
  double output;
  double outputMax;
  double outputMin;
};

#endif /** */
