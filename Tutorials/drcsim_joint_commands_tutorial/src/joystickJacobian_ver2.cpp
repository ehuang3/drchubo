#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
//#include <boost/bind.hpp>//ERROR?
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
//#include <osrf_msgs/JointCommands.h>
#include </usr/share/drcsim-2.5/ros/atlas_msgs/msg_gen/cpp/include/atlas_msgs/AtlasCommand.h>
#include </usr/share/sandia-hand-5.1/ros/sandia_hand_msgs/msg_gen/cpp/include/sandia_hand_msgs/SimpleGrasp.h>
#include <eigen3/Eigen/Dense>
#include <complex>
#include "atlasKin.h"
//#include <sensor_msgs/Joy>
//#include "joystick.h"
#include <cstdlib>

ros::Publisher pub_joint_commands_;
atlas_msgs::AtlasCommand jointcommands;//osrf_msgs::JointCommands jointcommands;
ros::Publisher pub_hand_commands;
sandia_hand_msgs::SimpleGrasp handcommands;
//osrf_msgs::JointCommands handcommands;
using namespace Eigen;
using namespace std;

atlasKin larm;


void atlasKin::getJointStates(const sensor_msgs::JointState::ConstPtr &_js){
 // cout << "Current: ";
  for (int b = 16;b<28;b++)
    //cout << _js->position[b] << " ";
  //cout << endl << "Command: ";
  // Make a moving filter to smooth inputs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (current[0] == 4){// this is set to be four in advance, as a first time through flag
  cout << "First Time" <<endl;
    for (int i=0;i<12;i++)
      current[i] = _js->position[i+16];
  }
  else{
    for (int i=0;i<12;i++){
      current[i] = (current[i]+_js->position[i+16])/2;
    }
  }
  // Get current transformation matrices ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
  trans(current);
  getJacobian(current);
  //ros::Duration(0.5).sleep();
  VectorXf l_cmd(6);
  VectorXf r_cmd(6);
  l_cmd = l_jacobian.inverse()*l_diff;
  r_cmd = r_jacobian.inverse()*r_diff;
  for (int a=0;a<6;a++){
    command[a] = 0.5*l_cmd(a)+current[a];
    command[a+6] = 0.5*r_cmd(a)+current[a+6];
    //cout << r_cmd[a] << " ";
    //cout << command[a] << " ";
  }
  //cout << endl;
  //cout << endl;
  setJointStates();
  l_diff << 0,0,0,0,0,0;
  r_diff << 0,0,0,0,0,0;
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
  /*jointcommands.name.push_back("atlas::back_lbz");
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
  jointcommands.name.push_back("atlas::r_arm_mwx");*/
  unsigned int n = 28;//jointcommands.name.size();
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
  float gains[28] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100000,50000,10000,10000,2500,5000,100000,50000,10000,10000,2500,5000};
  for (unsigned int i = 0; i < n; i++)
  {
    /*std::vector<std::string> pieces;
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
*/
    jointcommands.velocity[i]     = 0;
    jointcommands.effort[i]       = 0;
    if (i >15)
      jointcommands.k_effort[i]   = 255;
    else
      jointcommands.k_effort[i]	  = 0;
    jointcommands.kp_velocity[i]  = 0;
    jointcommands.kp_position[i]  = gains[i];

  }

  printf("Starting subscriptions\n");

  ros::Subscriber getJointStates = rosnode->subscribe("/atlas/joint_states",1,&atlasKin::getJointStates,&larm);
  ros::Subscriber joy_sub = rosnode->subscribe<sensor_msgs::Joy>("/joy",10,&atlasKin::joystick_J,&larm);
  pub_joint_commands_ = rosnode->advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1, true);
  pub_hand_commands = rosnode->advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/r_hand/simple_grasp", 1, true);
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~            ~~~~~~~~~~~~~~~~~
  float start[28] = {0,0.00062,0.00017,-0.00107,0.303711,0.03325,-0.24969,0.50624,-0.24686,-0.04443,-030352,-0.06482,-0.25276,0.52750,-0.27308,0.050106,0.29955,1.30236,-2.00036,-0.49928,0,-0.00094,-0.45,0.6,1.5708,-1.1708,-1.708,0};
 // float i[28] = {1000,1000,0,0,2000,2000,1000,1000,500,500,2000,2000,1000,1000,500,500,2000,2000,1000,1000,500,500,2000,2000,1000,1000,500,500};
  for (int a = 0; a<28; a++){
    jointcommands.position[a] = start[a];
    //jointcommands.ki_position[a] = i[a];
    jointcommands.kp_position[a] = jointcommands.kp_position[a]*50;
    //jointcommands.kd_position[a] = 0;//jointcommands.kp_position[a]/50;
  }
  jointcommands.kp_position[0] = 500;// jointcommands.kp_position[0]*3;
  jointcommands.kd_position[0] = 100;
  pub_joint_commands_.publish(jointcommands);
  ros::spinOnce();
  ros::Duration(1.0).sleep();

  larm.current[0] = 4;
  //larm.goal.resize(4,4);
  //larm.goal << 0,0,1, 0,-1,0,0,0.394,0,-1,0,0.295,0,0,0,1;
  larm.l_diff.resize(6);
  larm.l_diff << 0,0,0,0,0,0;
  larm.r_diff.resize(6);
  larm.r_diff << 0,0,0,0,0,0;
  while(ros::ok()){
    ros::spinOnce();      
  }
  return 0;
}
