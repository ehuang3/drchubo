#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
//#include <boost/bind.hpp>//ERROR?
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <eigen3/Eigen/Dense>
#include <complex>
#include "atlasKin.h"
//#include <sensor_msgs/Joy>
//#include "joystick.h"
#include <cstdlib>

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;

using namespace Eigen;
using namespace std;

atlasKin larm;


void atlasKin::getJointStates(const sensor_msgs::JointState::ConstPtr &_js){
  cout << "Current: ";
  for (int b = 16;b<22;b++)
    cout << _js->position[b] << " ";
  cout << endl << "Command: ";
  //cout << "getJointStates" << endl;
  //cout << endl;
  if (current[0] == 4){// this is set to be four in advance, as a first time through flag
  cout << "First Time" <<endl;
    for (int i=0;i<6;i++)
      current[i] = _js->position[i+16];
  }
  else{
    for (int i=0;i<6;i++){
      current[i] = (current[i]+_js->position[i+16])/2;
      //cout << current[i] << " ";
    }
  }
  //cout << endl;  
  trans(current);

  /*MatrixXf goal(4,4);
  goal = A06;
  goal(0,3) = goal(0,3)+larm.X;
  goal(1,3) = goal(1,3)+larm.Y;*/
//cout << "Current:" << endl <<A06 << endl;
//cout << "Goal:" << endl << goal << endl;
  ros::Duration(0.5).sleep();
  calculate(goal);
  for (int a=0;a<6;a++)
    cout << command[a] << " ";
  cout << endl;
  setJointStates();
}
void atlasKin::setJointStates(){
  //cout << "setJointStates" << endl;
  //cout << "Actually is: ";
  for (int i = 0;i<6;i++){
    jointcommands.position[i+16] = command[i];
    //cout << command[i] << " ";
  }
  //cout << endl;
  pub_joint_commands_.publish(jointcommands);
}


int main(int argc, char** argv){

  //atlasKin larm;

  printf("Main begun\n");
  ros::init(argc, argv, "pub_joint_command_test");
printf("Next\n");
  ros::NodeHandle* rosnode = new ros::NodeHandle();

printf("Next\n");
  system("rosrun joy joy_node&");

  printf("ROS started\n");
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  // must match those inside AtlasPlugin
  jointcommands.name.push_back("atlas::back_lbz");
  jointcommands.name.push_back("atlas::back_mby");
  jointcommands.name.push_back("atlas::back_ubx");
  jointcommands.name.push_back("atlas::neck_ay");
  jointcommands.name.push_back("atlas::l_leg_uhz");
  jointcommands.name.push_back("atlas::l_leg_mhx");
  jointcommands.name.push_back("atlas::l_leg_lhy");
  jointcommands.name.push_back("atlas::l_leg_kny");
  jointcommands.name.push_back("atlas::l_leg_uay");
  jointcommands.name.push_back("atlas::l_leg_lax");
  jointcommands.name.push_back("atlas::r_leg_uhz");
  jointcommands.name.push_back("atlas::r_leg_mhx");
  jointcommands.name.push_back("atlas::r_leg_lhy");
  jointcommands.name.push_back("atlas::r_leg_kny");
  jointcommands.name.push_back("atlas::r_leg_uay");
  jointcommands.name.push_back("atlas::r_leg_lax");
  jointcommands.name.push_back("atlas::l_arm_usy");
  jointcommands.name.push_back("atlas::l_arm_shx");
  jointcommands.name.push_back("atlas::l_arm_ely");
  jointcommands.name.push_back("atlas::l_arm_elx");
  jointcommands.name.push_back("atlas::l_arm_uwy");
  jointcommands.name.push_back("atlas::l_arm_mwx");
  jointcommands.name.push_back("atlas::r_arm_usy");
  jointcommands.name.push_back("atlas::r_arm_shx");
  jointcommands.name.push_back("atlas::r_arm_ely");
  jointcommands.name.push_back("atlas::r_arm_elx");
  jointcommands.name.push_back("atlas::r_arm_uwy");
  jointcommands.name.push_back("atlas::r_arm_mwx");
  unsigned int n = jointcommands.name.size();
  jointcommands.position.resize(n);
  jointcommands.velocity.resize(n);
  jointcommands.effort.resize(n);
  jointcommands.kp_position.resize(n);
  jointcommands.ki_position.resize(n);
  jointcommands.kd_position.resize(n);
  jointcommands.kp_velocity.resize(n);
  jointcommands.i_effort_min.resize(n);
  jointcommands.i_effort_max.resize(n);

  for (unsigned int i = 0; i < n; i++)
  {
    std::vector<std::string> pieces;
    boost::split(pieces, jointcommands.name[i], boost::is_any_of(":"));

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p",
      jointcommands.kp_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i",
      jointcommands.ki_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d",
      jointcommands.kd_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointcommands.i_effort_min[i]);
    jointcommands.i_effort_min[i] = -jointcommands.i_effort_min[i];

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointcommands.i_effort_max[i]);

    jointcommands.velocity[i]     = 0;
    jointcommands.effort[i]       = 0;
    jointcommands.kp_velocity[i]  = 0;
  }

  printf("Starting subscriptions\n");

  ros::Subscriber getJointStates = rosnode->subscribe("/atlas/joint_states",1,&atlasKin::getJointStates,&larm);
  ros::Subscriber joy_sub = rosnode->subscribe<sensor_msgs::Joy>("/joy",10,&atlasKin::joystick,&larm);
  pub_joint_commands_ = rosnode->advertise<osrf_msgs::JointCommands>("/atlas/joint_commands", 1, true);
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~            ~~~~~~~~~~~~~~~~~
  float start[28] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.5708,0,0,0,0,0,0,0,0};
  for (int a = 0; a<28; a++)
    jointcommands.position[a] = start[a];
  pub_joint_commands_.publish(jointcommands);
  ros::spinOnce();
  ros::Duration(1.0).sleep();

  larm.current[0] = 4;
  larm.goal.resize(4,4);
  larm.goal << 0,0,1, 0,-1,0,0,0.394,0,-1,0,0.295,0,0,0,1;
  while(ros::ok()){
    ros::spinOnce();      
      
//cout << larm.X << "   " << larm.Y << endl;
     // cout <<  "Goal" << endl << goal << endl;
      
     // cout << "Command" << endl;
      
//for (int zz = 0; zz<6; zz++)
 //       cout << larm.command[zz] << " ";
   //   cout << endl;
    //cout << larm.command[0] << endl;
   }

    
  return 0;
}
