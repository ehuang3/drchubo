#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include </usr/share/drcsim-2.6/ros/atlas_msgs/msg_gen/cpp/include/atlas_msgs/AtlasCommand.h>
#include </usr/share/sandia-hand-5.1/ros/sandia_hand_msgs/msg_gen/cpp/include/sandia_hand_msgs/SimpleGrasp.h>
#include <eigen3/Eigen/Dense>
#include <complex>
#include "atlasKin.h"
#include <cstdlib>

ros::Publisher pub_joint_commands_;
atlas_msgs::AtlasCommand jointcommands;//osrf_msgs::JointCommands jointcommands;
ros::Publisher pub_hand_commands;
sandia_hand_msgs::SimpleGrasp handcommands;

using namespace Eigen;
using namespace std;

atlasKin atlas;//TODO switch to larm if need be

void atlasKin::getJointStates(const sensor_msgs::JointState::ConstPtr &_js){
  for (int b = 16;b<28;b++)
  for (int i=0;i<12;i++)
    current[i] = _js->position[i+16];
  // Get current transformation matrices ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
  trans(current);
  getJacobian(current);

  l_goalPosition << 0.0,0.4,0.0;
  l_goalRot << 1,0,0,0,1,0,0,0,1;
  r_goalPosition << 0.3,-0.3,-0.15;
  r_goalRot << 1,0,0,0,1,0,0,0,1;

  VectorXf l_cmd(6);
  VectorXf r_cmd(6);

  l_diff.resize(6);
  r_diff.resize(6);
  getError();
  l_cmd = l_jacobian.inverse()*l_diff;
  r_cmd = r_jacobian.inverse()*r_diff;
  for (int a=0;a<6;a++){
    command[a] = l_cmd(a)+current[a];
    command[a+6] = r_cmd(a)+current[a+6];
  }

  setJointStates();
  
  l_diff << 0,0,0,0,0,0;
  r_diff << 0,0,0,0,0,0;
}

void atlasKin::getError(){
  // Get the error for the right arm
  cout << "Right Current:" << RA06.block<3,1>(0,3).transpose() << endl;;
  cout << "   Right Goal:" << r_goalPosition.transpose() << endl;
  r_diff.head<3>() = r_goalPosition - RA06.block<3,1>(0,3);
  cout << "   Right Diff:" << r_diff.head<3>().transpose() << endl << endl;
  r_diff(3) = -(r_goalRot(1,2)-RA06(1,2))/(r_goalRot(0,0)-RA06(0,0));//(r_goalRot(2,1)-RA06(2,1)+RA06(1,2)-r_goalRot(1,2))/2;
  r_diff(4) = (r_goalRot(0,2)-RA06(0,2))/(r_goalRot(0,0)-RA06(0,0));//(RA06(0,2)-r_goalRot(0,2)+r_goalRot(2,0)-RA06(2,0))/2;
  r_diff(5) = -(r_goalRot(0,1)-RA06(0,1))/(r_goalRot(0,0)-RA06(0,0));//(r_goalRot(1,0)-RA06(1,0)+RA06(0,1)-r_goalRot(0,1))/2;
  r_diff.tail<3>() << 0,0,0;

  if (r_diff.head<3>().norm() > 0.003){
    r_diff = r_diff/r_diff.head<3>().norm();
    r_diff = 0.003*r_diff;
  }
  else {r_diff << 0,0,0,0,0,0;}

  // Repeat for Left arm
  cout << " Left Current:" << LA06.block<3,1>(0,3).transpose() << endl;

  cout << "    Left Goal:" << l_goalPosition.transpose() << endl;
  l_diff.head<3>() = l_goalPosition - LA06.block<3,1>(0,3);
  cout << "    Left Diff:" << l_diff.head<3>().transpose() << endl << endl;
  l_diff(3) = -(l_goalRot(1,2)-LA06(1,2))/(l_goalRot(0,0)-LA06(0,0));//(l_goalRot(2,1)-LA06(2,1)+LA06(1,2)-l_goalRot(1,2))/2;
  l_diff(4) = (l_goalRot(0,2)-LA06(0,2))/(l_goalRot(0,0)-LA06(0,0));//(LA06(0,2)-l_goalRot(0,2)+l_goalRot(2,0)-LA06(2,0))/2;
  l_diff(5) = -(l_goalRot(0,1)-LA06(0,1))/(l_goalRot(0,0)-LA06(0,0));//(l_goalRot(1,0)-LA06(1,0)+LA06(0,1)-l_goalRot(0,1))/2;
  l_diff.tail<3>() << 0,0,0;

  if (l_diff.head<3>().norm() > 0.003){
    l_diff = l_diff/l_diff.head<3>().norm();
    l_diff = 0.005*l_diff;
  }
  else {l_diff << 0,0,0,0,0,0;}
}

void atlasKin::setJointStates(){
  //cout << "setJointStates" << endl;
  //cout << "Actually is: ";
  for (int i = 0;i<12;i++){
    jointcommands.position[i+16] = command[i];
    //cout << command[i] << " ";
  }
  jointcommands.position[0] = jointcommands.position[0]+twist;
  //cout << endl;

  pub_joint_commands_.publish(jointcommands);

  handcommands.name = "cylindrical";
  handcommands.closed_amount = r_closed_amount;
 // cout << jointcommands.kd_position[0] << endl;
  pub_hand_commands.publish(handcommands);
}

int main(int argc, char** argv){

  printf("Main begun\n");
  ros::init(argc, argv, "pub_joint_command_test");
printf("Next\n");
  ros::NodeHandle* rosnode = new ros::NodeHandle();

  printf("ROS started\n");
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }
  unsigned int n = 28;
  jointcommands.position.resize(n);
  jointcommands.velocity.resize(n);
  jointcommands.effort.resize(n);
  jointcommands.k_effort.resize(n);
  jointcommands.kp_position.resize(n);
  jointcommands.ki_position.resize(n);
  jointcommands.kd_position.resize(n);
  jointcommands.kp_velocity.resize(n);
  jointcommands.i_effort_min.resize(n);
  jointcommands.i_effort_max.resize(n);

  float p_gains[28] = {500.0, 200000.0, 100000.0, 1000.0, 250.0, 5000.0, 100000.0, 50000.0, 45000.0, 15000.0, 250.0, 5000.0, 100000.0, 50000.0, 45000.0, 15000.0, 100000.0, 50000.0, 50000.0, 10000.0, 2500.0, 5000.0, 100000.0, 50000.0, 50000.0, 10000.0, 2500.0, 5000.0};
  float i_gains[28] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  float d_gains[28] = {100.0, 2.0, 1.0, 1.0, 0.01, 1.0, 10.0, 10.0, 2.0, 1.0, 0.01, 1.0, 10.0, 10.0, 2.0, 1.0, 3.0, 20.0, 3.0, 3.0, 0.1, 0.2, 3.0, 20.0, 3.0, 3.0, 0.1, 0.2};
  for (unsigned int i = 0; i < n; i++)
  {
    jointcommands.velocity[i]     = 0;
    jointcommands.effort[i]       = 0;
    if (i >15)
      jointcommands.k_effort[i]   = 255;
    else
      jointcommands.k_effort[i]	  = 0;
    jointcommands.kp_velocity[i]  = 0;
    jointcommands.kp_position[i]  = p_gains[i];
    jointcommands.ki_position[i]  = i_gains[i];
    jointcommands.kd_position[i]  = d_gains[i];

  }

  printf("Starting subscriptions\n");

  ros::Subscriber getJointStates = rosnode->subscribe("/atlas/joint_states",1,&atlasKin::getJointStates,&atlas);
  pub_joint_commands_ = rosnode->advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1, true);
  pub_hand_commands = rosnode->advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/r_hand/simple_grasp", 1, true);

  float start[28] = {0,0.00062,0.00017,-0.00107,0.303711,0.03325,-0.24969,0.50624,-0.24686,-0.04443,-0.30352,-0.06482,-0.25276,0.52750,0,0,0.299873,-1.303457,2.000789,0.498209,0.0002929,-0.004480,0.29952,1.30444,2.000786,-0.498207,0.0002915,0.0044848};
  for (int a = 0; a<28; a++){
    jointcommands.position[a] = start[a];
    jointcommands.kp_position[a] = jointcommands.kp_position[a]*10;
    jointcommands.kd_position[a] = jointcommands.kd_position[a]*10;
  }
  jointcommands.kp_position[0] = 500;// jointcommands.kp_position[0]*3;
  jointcommands.kd_position[0] = 100;
  pub_joint_commands_.publish(jointcommands);
  ros::spinOnce();
  ros::Duration(1.0).sleep();

  //larm.goal.resize(4,4);
  //larm.goal << 0,0,1, 0,-1,0,0,0.394,0,-1,0,0.295,0,0,0,1;
  //larm.l_diff.resize(6);
  //larm.l_diff << 0,0,0,0,0,0;
  //larm.r_diff.resize(6);
  //larm.r_diff << 0,0,0,0,0,0;
  while(ros::ok()){
    ros::spinOnce();      
  }
  return 0;
}
