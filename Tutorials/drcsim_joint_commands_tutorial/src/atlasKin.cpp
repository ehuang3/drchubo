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
  //cout << l_jacobian << endl << endl;
}

/*void atlasKin::calculate(MatrixXf goal){
 //cout << "Calculator" << endl << endl;
  
  MatrixXf A16(4,4);
  A16 = goal;
 
  A16(1,3) = A16(1,3)-0.075;
  A16(2,3) = A16(2,3)-0.036;

  MatrixXf A61(4,4);
  A61 = A16.inverse();

  float mag = A16.block<3,1>(0,3).norm();


  float tmp = (mag*mag-0.306276*0.306276-0.246343*0.246343)/(-2*0.306276*0.246343);
  //cout << "tmp: " << tmp << endl;
  if (tmp > 1)
    tmp = 1;
  if (tmp < -1)
    tmp = -1;
  float q4 = acos(tmp);
  q4 = q4 - atan2(0.013,0.306)-atan2(0.013,0.246);

  q4 = 3.14159-q4;
 // if (q4 != q4)
//    q4 = 1.5708;
  
  float q5 = asin(A61(2,3)/(0.013*cos(q4)-0.013-0.306*sin(q4)));

  float A = 0.246+0.306*cos(q4)+0.013*sin(q4);
  float B = (0.013*cos(q5)-0.013*cos(q4)*cos(q5)+0.306*sin(q4)*cos(q5));
  float C = -0.013*cos(q5)-0.306*sin(q4)*cos(q5)+0.013*cos(q4)*cos(q5);
  float D = (0.246+0.306*cos(q4)+0.013*sin(q4));

  float C6 = (A61(0,3)-(A/C)*A61(1,3))/(B-A*D/C);
  float S6 = (A61(0,3)-B*C6)/A;

  float q6 = atan2(S6,C6);

  command[3] = q4;
  command[4] = q5;
  command[5] = q6;
  
  trans(command);

  MatrixXf A01a(4,4);
  A01a << 0,1,0,0,-0.866025,0,0.5,0.075,0.5,0,0.866025,0.036,0,0,0,1;

  MatrixXf A1a3(4,4);
  A1a3 = A01a.inverse()*goal*A6.inverse()*A5.inverse()*A4.inverse();
  
  //float b = -2*cos(atan2(.036,.075))*A1a3(2,2);
  //float c = A1a3(2,2)*A1a3(2,2)-sin(atan2(0.036,0.075))*sin(atan2(0.036,0.075));
  //float q2 = -acos((-b+sqrt(b*b-4*c))/2);
  float a = 1+(0.25/(0.866025*0.866025));
  float b = -A1a3(2,2)/(0.866025*0.866025);
  float c = (A1a3(2,2)*A1a3(2,2))/(0.866025*0.866025) -1;
  tmp = (-b+sqrt(b*b-4*a*c))/(2*a);
  if (tmp > 1)
    tmp = 1;
  float q2 = acos(tmp);
  tmp = A1a3(2,0)/(.866025*cos(q2)-0.5*sin(q2));
  if (tmp > 1)
    tmp = 1;
  else if  (tmp < -1)
    tmp = -1;
  std::complex<double> t3 = acos(tmp);//TODO CHECK
  float q3 = t3.real();

  tmp = A1a3(1,2)/(.866025*cos(q2)-0.5*sin(q2));
  if (tmp > 1)
    tmp = 1;
  else if  (tmp < -1)
    tmp = -1;
  std::complex<double> t1 = asin(A1a3(1,2)/(.866025*cos(q2)-0.5*sin(q2)));
  float q1 = -t1.real();
 

  command[0] = q1;
  command[1] = q2;
  command[2] = q3;
 
  trans(command);
}

void atlasKin::joystick(const sensor_msgs::Joy::ConstPtr& joy){
  X = 0;
  Y = 0;
  Z = 0;
  if (joy->buttons[12] == 1){
    goal << 0,-0.198669,0.980667,0.0514554,-1,0,0,.393999,0,-0.98066,-0.198669,0.289837,0,0,0,1;
    //X = 0.005;
    //cout << "Triangle" << endl;
  }
  if (joy->buttons[13] == 1){
    goal << 0,0,1, 0,-1,0,0,0.394,0,-1,0,0.295,0,0,0,1;
    //Y =  -0.005;
    //cout << "Circle" << endl;
  }
  if (joy->buttons[14] == 1){
    goal << -0.162394,-0.242834,0.9638,0.151277,-0.98613,0.073230,-0.0148733,0.210361,-0.343257,-0.967267,-0.251426,0.531342,0,0,0,1;
    //X = -0.005;
    //cout << "X" << endl;
  }
  if (joy->buttons[15] == 1){
    goal << 0,0,01,0,-.960834,-.277121,0,.448933,0.277121,-0.960834,0,0.357064,0,0,0,1;
    //Y = 0.005;
    //cout << "Square" << endl;
  }
}*/
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
