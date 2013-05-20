#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <eigen3/Eigen/Dense>
#include <complex>
#include "atlasKin.h"
#include <sensor_msgs/Joy.h>

using namespace Eigen;
using namespace std;

void atlasKin::trans(float t[]){
  LA1 << sin(t[0]),cos(t[0]),0,0,-0.866025*cos(t[0]),0.866025*sin(t[0]),0.5,0.075,0.5*cos(t[0]),-0.5*sin(t[0]),0.866025,0.036,0,0,0,1;   
  LA2 << 0.5*cos(t[1])+0.866025*sin(t[1]),0.866025*cos(t[1])-0.5*sin(t[1]),0,0,0,0,1,0,0.866025*cos(t[1])-0.5*sin(t[1]),-0.866025*sin(t[1])-0.5*cos(t[1]),0,0,0,0,0,1;
  LA3 << cos(t[2]),-sin(t[2]),0,0,0,0,-1,-0.306,sin(t[2]),cos(t[2]),0,0,0,0,0,1;
  LA4 << cos(t[3]),-sin(t[3]),0,0.013,0,0,1,0,-sin(t[3]),-cos(t[3]),0,0,0,0,0,1;
  LA5 << cos(t[4]),-sin(t[4]),0,-0.013,0,0,-1,-0.246,sin(t[4]),cos(t[4]),0,0,0,0,0,1;
  LA6 << cos(t[5]),-sin(t[5]),0,0,0,0,1,0,-sin(t[5]),-cos(t[5]),0,0,0,0,0,1;
  LA06 = LA1*LA2*LA3*LA4*LA5*LA6;

  RA1 << sin(t[6]),cos(t[6]),0,0,0.866025*cos(t[6]), -0.866025*sin(t[6]),0.5,-0.075,0.5*cos(t[6]),-0.5*sin(t[6]), -0.866025, 0.036,0,0,0,1; 
  RA2 << 0.5*cos(t[7]) - 0.866025*sin(t[7]), -0.866025*cos(t[7]) - 0.5*sin(t[7]),0,0,0,0,1,0,-0.866025*cos(t[7])-0.5*sin(t[7]),0.866025*sin(t[7])-0.5*cos(t[7]),0,0,0,0,0,1;
  RA3 << cos(t[8]), -sin(t[8]),0,0,0,0,-1,0.306,sin(t[8]),cos(t[8]),0,0,0,0,0,1;
  RA4 << cos(t[9]), -sin(t[9]),0,0.013,0,0,1,0,-sin(t[9]),-cos(t[9]),0,0,0,0,0,1;
  RA5 << cos(t[10]), -sin(t[10]),0, -0.013,0,0,-1,0.246,sin(t[10]),cos(t[10]),0,0,0,0,0,1;
  RA6 << cos(t[11]), -sin(t[11]),0,0,0,0,1,0,-sin(t[11]),-cos(t[11]),0,0,0,0,0,1;
  RA06 = RA1*RA2*RA3*RA4*RA5*RA6;
}
void atlasKin::getJacobian(float t[]){
  l_jacobian.resize(6,6);
  r_jacobian.resize(6,6);
  trans(t);
  Matrix4f * l_htms[6] = {&LA1,&LA2,&LA3,&LA4,&LA5,&LA6};
  Matrix4f * r_htms[6] = {&RA1,&RA2,&RA3,&RA4,&RA5,&RA6};
  Matrix4f lfk, rfk;
  lfk = Matrix4f::Identity(4,4);
  rfk = Matrix4f::Identity(4,4);
  for(int i=0;i<6;i++){
    lfk = lfk* *l_htms[i];
    rfk = rfk* *r_htms[i];
    l_jacobian.block<3,1>(0,i) = lfk.block<3,1>(0,3).cross(lfk.block<3,1>(0,2));
    r_jacobian.block<3,1>(0,i) = rfk.block<3,1>(0,3).cross(rfk.block<3,1>(0,2));
    l_jacobian.block<3,1>(3,i) = lfk.block<3,1>(0,2);
    r_jacobian.block<3,1>(3,i) = rfk.block<3,1>(0,2);    
  }
}

void atlasKin::joystick_J(const sensor_msgs::Joy::ConstPtr& joy){
  r_diff.resize(6);
  twist = 0;
  //r_diff.resize(6);
  if (joy->buttons[4] == 1){
    r_diff << 0,0,0.005,0,0,0;
    //cout << "UP" << endl;
  }
  if (joy->buttons[5] == 1){
    twist = -0.0174;
  }
  if (joy->buttons[6] == 1){
    r_diff << 0,0,-0.005,0,0,0;
    //cout << "DOWN" << endl;
  }
  if (joy->buttons[7] == 1){
    twist = 0.0174;
  }
  if (joy->buttons[8] == 1){
    r_closed_amount = 1;
  }
  if (joy->buttons[8] == 0){
    r_closed_amount = 0;
  }
  if (joy->buttons[12] == 1 && joy->buttons[10] == 0){
    r_diff << 0.005,0,0,0,0,0;
    //cout << "Triangle" << endl;
  }
  if (joy->buttons[13] == 1 && joy->buttons[10] == 0){
    r_diff << 0,-0.005,0,0,0,0;
    //cout << "Circle" << endl;
  }
  if (joy->buttons[14] == 1 && joy->buttons[10] == 0){
    r_diff << -0.005,0,0,0,0,0;
    //cout << "X" << endl;
  }
  if (joy->buttons[15] == 1 && joy->buttons[10] == 0){
    r_diff << 0,0.005,0,0,0,0;
    //cout << "Square" << endl;
  }
  if (joy->buttons[12] == 1 && joy->buttons[10] == 1){
    r_diff << 0,0,0,0.005,0,0;
    //cout << "Triangle" << endl;
  }
  if (joy->buttons[13] == 1 && joy->buttons[10] == 1){
    r_diff << 0,0,0,0,0,-0.005;
    //cout << "Circle" << endl;
  }
  if (joy->buttons[14] == 1 && joy->buttons[10] == 1){
    r_diff << 0,0,0,-0.005,0,0;
    //cout << "X" << endl;
  }
  if (joy->buttons[15] == 1 && joy->buttons[10] == 1){
    r_diff << 0,0,0,0,0,0.005;
    //cout << "Square" << endl;
  }
}
