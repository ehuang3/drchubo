#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;

class limb {
public:
  MatrixXf trans;// Contains all 6 4x4 matrices
  MatrixXf fk;
  MatrixXf J;
  MatrixXf pose;
  VectorXf goal;
  VectorXf goalPose;
  VectorXf step;

  MatrixXf jacobian(float q[]);

} lleg, rleg, larm,rarm;

void transforms(float q[]){

  lleg.trans.resize(24,4);
  rleg.trans.resize(24,4);
  larm.trans.resize(24,4);
  rarm.trans.resize(24,4);

  // Define homogeneous transformation matrices for the left leg  
  lleg.trans << -sin(q[4]), -cos(q[4]), 0, 0, cos(q[4]) ,  -sin(q[4]), 0, 0.089, 0,  0, 1, 0, 0,0, 0, 1,
    cos(q[5]), -sin(q[5]),0, 0,0, 0, -1, 0, sin(q[5]), cos(q[5]), 0, 0, 0, 0, 0, 1,
    0, 0, 1, 0, -cos(q[6]), sin(q[6]),0, -0.05,-sin(q[6]), -cos(q[6]), 0, -0.05, 0, 0, 0,1,
    cos(q[7]), -sin(q[7]), 0, 0.374, sin(q[7]),  cos(q[7]), 0,   -0.05, 0,0, 1,0,0,0, 0, 1,
    cos(q[8]), -sin(q[8]), 0, 0.422, sin(q[8]),  cos(q[8]), 0, 0,0, 0, 1, 0,0, 0, 0, 1,
    cos(q[9]), -sin(q[9]),0, 0,0, 0, -1, 0,sin(q[9]), cos(q[9]),0, 0,0, 0,  0, 1;

  // Define homogeneous transformation matrices for the right leg  
  rleg.trans << -sin(q[10]), -cos(q[10]), 0, 0, cos(q[10]) ,  -sin(q[10]), 0, -0.089, 0,  0, 1, 0, 0,0, 0, 1,
    cos(q[11]), -sin(q[11]),0, 0,0, 0, -1, 0, sin(q[11]), cos(q[11]), 0, 0, 0, 0, 0, 1,
    0, 0, 1, 0, -cos(q[12]), sin(q[12]),0, -0.05,-sin(q[12]), -cos(q[12]), 0, -0.05, 0, 0, 0,1,
    cos(q[13]), -sin(q[13]), 0, 0.374, sin(q[13]),  cos(q[13]), 0,   -0.05, 0,0, 1,0,0,0, 0, 1,
    cos(q[14]), -sin(q[14]), 0, 0.422, sin(q[14]),  cos(q[14]), 0, 0,0, 0, 1, 0,0, 0, 0, 1,
    cos(q[15]), -sin(q[15]),0, 0,0, 0, -1, 0,sin(q[15]), cos(q[15]),0, 0,0, 0,  0, 1;

  // Define homogeneous transformation matrices for the left arm
  larm.trans << cos(q[16]),-sin(q[16]),0, 0,0.866*sin(q[16]), 0.866*cos(q[16]),0.5, 0,-0.5*sin(q[16]),-0.5*cos(q[16]), 0.866, 0,0, 0,0, 1,
    0, 0,1,0,0.5*sin(q[17]) + 0.866*cos(q[17]),0.5*cos(q[17]) - 0.866*sin(q[17]), 0, 0,0.5*cos(q[17]) - 0.866*sin(q[17]),-0.5*sin(q[17] - 0.866*cos(q[17])),   0, 0,0,0,0,1,
    0,0,-1, 0.185,sin(q[18]),cos(q[18]),0,0,cos(q[18]),-sin(q[18]),0,0,0,0,0,1,
    0,0,1,0,sin(q[19]), cos(q[19]),0,-0.013,-cos(q[19]), sin(q[19]), 0, -0.121,0,0,0,1,
    0,0,-1, 0.188,sin(q[20]),cos(q[20]),0, 0.013,cos(q[20]),-sin(q[20]), 0,0,0,0,0,1,
    0,0,1,0,sin(q[21]),cos(q[21]),0,0,-cos(q[21]),sin(q[21]),0, -0.058,0,0,0,1;

  // Define homogeneous transformation matrices for the right arm
  // Due to unconventional notation, this is probably wrong
  rarm.trans << -cos(-q[22]),sin(-q[22]),0, 0,-0.866*sin(-q[22]), -0.866*cos(-q[22]),-0.5, 0,-0.5*sin(-q[22]),-0.5*cos(-q[22]),0.866, 0,0,0,0, 1,
    0, 0,1,0,0.5*sin(-q[23]) + 0.866*cos(-q[23]),0.5*cos(-q[23]) - 0.866*sin(-q[23]), 0, 0,0.5*cos(-q[23]) - 0.866*sin(-q[23]),-0.5*sin(-q[23]) - 0.866*cos(-q[23]),   0, 0,0,0,0,1,
    0,0,-1, 0.185,sin(-q[24]),cos(-q[24]),0,0,cos(-q[24]),-sin(-q[24]),0,0,0,0,0,1,
    0,0,1,0,sin(-q[25]), cos(-q[25]),0,-0.013,-cos(-q[25]), sin(-q[25]), 0, -0.121,0,0,0,1,
    0,0,-1, 0.188,sin(-q[26]),cos(-q[26]),0, 0.013,cos(-q[26]),-sin(-q[26]), 0,0,0,0,0,1,
    0,0,1,0,sin(-q[27]),cos(-q[27]),0,0,-cos(-q[27]),sin(-q[27]),0, -0.058,0,0,0,1;

}

  MatrixXf limb::jacobian(float q[]){ 

    MatrixXf temp(4,4);
    temp << MatrixXf::Identity(4,4);
    J.resize(6,6);

    transforms(q);
    //fk.resize(4,4);

    fk.resize(4,4);
    fk << MatrixXf::Identity(4,4);

    for (int i=0;i<6;i++){
      fk = fk*trans.block<4,4>(4*i,0);
      for (int j=i;j<6;j++){
         temp = temp*trans.block<4,4>(4*j,0);
      }
      temp = temp.inverse();

      J.block<3,1>(0,i) << temp.block<3,1>(0,3).cross(fk.block<3,1>(0,2));
      J.block<3,1>(3,i) << fk.block<3,1>(0,2);
      temp << MatrixXf::Identity(4,4);
    }
    return J;
  }

void initJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
  float current[28];
  static ros::Time startTime = ros::Time::now();
  // for testing round trip time
  jointcommands.header.stamp = _js->header.stamp;
  for (int z = 0; z<28; z++)
    current[z] = _js->position[z];
  float init[28] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.507,0.3927,-0.7535,0,0,0,1.507,0.3927,-0.7535,0};
  for (int a =0;a<28;a++){
    jointcommands.position[a] = init[a];
  }
  pub_joint_commands_.publish(jointcommands);

  // Define the HTMs for the current joint angles
  transforms(current);

  // Define goal poses for each joint
  lleg.goalPose.resize(6);
  rleg.goalPose.resize(6);
  larm.goalPose.resize(6);
  rarm.goalPose.resize(6);

  lleg.jacobian(current);
  rleg.jacobian(current);
  larm.jacobian(current);
  rarm.jacobian(current);

  lleg.goalPose.segment<3>(0) = lleg.fk.block<3,1>(0,3) + lleg.goal.segment<3>(0);
  lleg.goalPose.segment<3>(3) = lleg.fk.block<3,1>(0,2) + lleg.goal.segment<3>(3);
  rleg.goalPose.segment<3>(0) = rleg.fk.block<3,1>(0,3) + rleg.goal.segment<3>(0);
  rleg.goalPose.segment<3>(3) = rleg.fk.block<3,1>(0,2) + rleg.goal.segment<3>(3);
  larm.goalPose.segment<3>(0) = larm.fk.block<3,1>(0,3) + larm.goal.segment<3>(0);
  larm.goalPose.segment<3>(3) = larm.fk.block<3,1>(0,2) + larm.goal.segment<3>(3);
  rarm.goalPose.segment<3>(0) = rarm.fk.block<3,1>(0,3) + rarm.goal.segment<3>(0);
  rarm.goalPose.segment<3>(3) = rarm.fk.block<3,1>(0,2) + rarm.goal.segment<3>(3);


}

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js){

  float current[28];
  static ros::Time startTime = ros::Time::now();
  // for testing round trip time
  jointcommands.header.stamp = _js->header.stamp;

  for (int z = 0; z<28; z++)
    current[z] = (current[z]+_js->position[z])/2;

  // calculate jacobian for each limb

  lleg.jacobian(current);
  rleg.jacobian(current);
  larm.jacobian(current);
  rarm.jacobian(current);

  // Define the current pose
  VectorXf currentPose(24);
  currentPose.segment<3>(0) = lleg.fk.block<3,1>(0,3);
  currentPose.segment<3>(3) = lleg.fk.block<3,1>(0,2);
  currentPose.segment<3>(6) = rleg.fk.block<3,1>(0,3);
  currentPose.segment<3>(9) = rleg.fk.block<3,1>(0,2);
  currentPose.segment<3>(12) = larm.fk.block<3,1>(0,3);
  currentPose.segment<3>(15) = larm.fk.block<3,1>(0,2);
  currentPose.segment<3>(18) = rarm.fk.block<3,1>(0,3);
  currentPose.segment<3>(21) = rarm.fk.block<3,1>(0,2);

//~~~~~~~~~~~Find distance to goal
lleg.goal.block<3,1>(0,0)=lleg.goalPose.segment<3>(0) - currentPose.segment<3>(0);
rleg.goal.block<3,1>(0,0)=rleg.goalPose.segment<3>(0) - currentPose.segment<3>(6);
larm.goal.block<3,1>(0,0)=larm.goalPose.segment<3>(0) - currentPose.segment<3>(12);
rarm.goal.block<3,1>(0,0)=rarm.goalPose.segment<3>(0) - currentPose.segment<3>(18);

//~~~~~~~~~~~Redefine steps on each iteration
  lleg.step << (0.0001/lleg.goal.block<3,1>(0,0).norm())*lleg.goal;
  if (lleg.goal.block<3,1>(0,0).norm() == 0)
    lleg.step << 0, 0, 0,0,0,0;
  rleg.step << (0.0001/rleg.goal.block<3,1>(0,0).norm())*rleg.goal;
  if (rleg.goal.block<3,1>(0,0).norm() == 0)
    rleg.step << 0, 0, 0,0,0,0;
  larm.step<< (0.0001/larm.goal.block<3,1>(0,0).norm())*larm.goal;
  if (larm.goal.block<3,1>(0,0).norm() == 0)
    larm.step << 0, 0, 0,0,0,0;
  rarm.step << (0.0001/rarm.goal.block<3,1>(0,0).norm())*rarm.goal;
  if (rarm.goal.block<3,1>(0,0).norm() == 0)
    rarm.step << 0, 0, 0,0,0,0;

  float a=rarm.goalPose.segment<3>(0).norm();
  float b=currentPose.segment<18>(0).norm();
  ros::Duration(0.5).sleep();

  if (rarm.step.norm() < .005){//std::abs(a-b)){//this is wrong, do this better

    // delQ = inverse jacobian multiplied by step.
    VectorXf delQ(24); 
    MatrixXf temp(6,6);

    temp = lleg.J.inverse();
    delQ.segment<6>(0) = temp*lleg.step;
    temp =  rleg.J.inverse();
    delQ.segment<6>(6) = temp*rleg.step;
    temp = larm.J.inverse();
    delQ.segment<6>(12) = temp*larm.step;
    temp = rarm.J.inverse();
    delQ.segment<6>(18) = temp*rarm.step;
    for (int s=0; s<24;s++){
      while(delQ(s) > 3.1416)
        delQ(s) = delQ(s)-3.1416;
      while(delQ(s) < -3.1416)
        delQ(s) = delQ(s)+3.1416;
    }

    std::cout << "Current:\t" << currentPose.segment<3>(18).transpose() << std::endl;
    std::cout << "     Goal:\t" << rarm.goalPose.segment<3>(0).transpose() << std::endl;
    // cmd = current + delQ
    float temp1, temp2;
    for (int q = 0;q<22;q++)//TODO this is just temporary to hold body fixed
      jointcommands.position[q] = 0;//current[q];
    for (int q = 22;q<28;q++){//fix these lines
      temp1=current[q];
      temp2=delQ[q-4];
      if (temp2 != temp2)
          temp2 = 0.01745;
      jointcommands.position[q] = current[q]+delQ[q-4];

    }
    printf("\n\n");
    pub_joint_commands_.publish(jointcommands);
  }
  else {printf("DONE\n");}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_joint_command_test");

  ros::NodeHandle* rosnode = new ros::NodeHandle();

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

  
  // ros topic subscribtions
  ros::SubscribeOptions jointStatesINIT =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, initJointStates,
    ros::VoidPtr(), rosnode->getCallbackQueue());

//~~~~~~~~~~~~~Create first subscriber ~~~~~~~~~~~~~~~~~~~~~
  jointStatesINIT.transport_hints = ros::TransportHints().unreliable();

  ros::Subscriber initJointStates = rosnode->subscribe(jointStatesINIT);

  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Define 4 goals;  
  lleg.goal.resize(6);
  rleg.goal.resize(6);
  larm.goal.resize(6);
  rarm.goal.resize(6);
  
  lleg.goal << 0.00,0.00,0,0,0,0;
  rleg.goal << 0,0,0,0,0,0;
  larm.goal << 0,0,0,0,0,0;
  rarm.goal << 0.04,0.010,0.015,0,0,0;
  printf("\nWaiting on initial pose\n");
  while(lleg.goalPose.size() == 0)
    ros::spinOnce();
  printf("Initial pose set\n");

//~~~~~~~~~~~~~Destroy first subscriber ~~~~~~~~~~~~~~~~~~~~~

//ros::Subscriber::shutdown();
  initJointStates.shutdown();
//~~~~~~~~~~~~~Trajectory Stuff~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ros topic subscribtions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, SetJointStates,
    ros::VoidPtr(), rosnode->getCallbackQueue());


  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();

  ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);
  // ros::Subscriber subJointStates =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetJointStates);


  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);
  // Divide into steps
  lleg.step.resize(6);
  rleg.step.resize(6);
  larm.step.resize(6);
  rarm.step.resize(6);
  printf("Generating steps\n");
  lleg.step << (0.0001/lleg.goal.block<3,1>(0,0).norm())*lleg.goal;
  if (lleg.goal.block<3,1>(0,0).norm() == 0)
    lleg.step << 0, 0, 0,0,0,0;
  //std::cout << lleg.step << std::endl;
  rleg.step << (0.0001/rleg.goal.block<3,1>(0,0).norm())*rleg.goal;
  if (rleg.goal.block<3,1>(0,0).norm() == 0)
    rleg.step << 0, 0, 0,0,0,0;
  //std::cout << rleg.goal.block<3,1>(0,0).norm()<< std::endl;
  larm.step<< (0.0001/larm.goal.block<3,1>(0,0).norm())*larm.goal;
  if (larm.goal.block<3,1>(0,0).norm() == 0)
    larm.step << 0, 0, 0,0,0,0;
  //std::cout << larm.step << std::endl;
  rarm.step << (0.0001/rarm.goal.block<3,1>(0,0).norm())*rarm.goal;
  if (rarm.goal.block<3,1>(0,0).norm() == 0)
    rarm.step << 0, 0, 0,0,0,0;
  //std::cout << rarm.step << std::endl;
  printf("Spin Time\n");
  ros::spin();

  return 0;
}
