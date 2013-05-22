#ifndef ATLAS_H
#define ATLAS_H

#include <sensor_msgs/Joy.h>

using namespace Eigen;

class atlasKin {
public:

  //Constructor
  atlasKin():X(0), Y(0), Z(0){}
  Matrix4f LA1;
  Matrix4f LA2;
  Matrix4f LA3;
  Matrix4f LA4;
  Matrix4f LA5;
  Matrix4f LA6;
  Matrix4f LA06;
  Matrix4f RA1;
  Matrix4f RA2;
  Matrix4f RA3;
  Matrix4f RA4;
  Matrix4f RA5;
  Matrix4f RA6;
  Matrix4f RA06;

  Vector3f l_goalPosition;
  Vector3f r_goalPosition;

  Matrix3f l_goalRot;
  Matrix3f r_goalRot;

  float l_closed_amount;
  float r_closed_amount;
  float twist;

  Matrix4f l_goal;
  Matrix4f r_goal;
  VectorXf l_diff;
  VectorXf r_diff;
  MatrixXf l_jacobian;
  MatrixXf r_jacobian;

  float command[12];
  float current[12];

  float X;
  float Y;
  float Z;
 

  void trans(float []);
  void strans(float []);
  void getJacobian(float []);
  void calculate(MatrixXf goal);
  void getJointStates(const sensor_msgs::JointState::ConstPtr &_js);
  void setJointStates();
  void joystick(const sensor_msgs::Joy::ConstPtr& joy);
  void joystick_J(const sensor_msgs::Joy::ConstPtr& joy);
  void getError();

  void fastrakDifferential();
};

#endif
