#include <math.h>
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
float current[28];

class limb {
public:
  MatrixXf trans;// Contains all 6 4x4 matrices
  MatrixXf fk;
  MatrixXf J;
  MatrixXf pose;

  MatrixXf jacobian(){ 

    //trans.resize(24,4);

    MatrixXf temp(4,4);
    temp << MatrixXf::Identity(4,4);
    J.resize(6,6);

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
} lleg, rleg, larm,rarm;

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
  static ros::Time startTime = ros::Time::now();
  // for testing round trip time
  jointcommands.header.stamp = _js->header.stamp;
  for (int z = 0; z<28; z++){
    current[z] = _js->position[z];
  }
}

void trans(float q[]){

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

VectorXf fk(float current[]){
  VectorXf currentPose(24);
  MatrixXf llegFk(4,4);
  MatrixXf rlegFk(4,4);
  MatrixXf larmFk(4,4);
  MatrixXf rarmFk(4,4);

  llegFk << MatrixXf::Identity(4,4);
  rlegFk << MatrixXf::Identity(4,4);
  larmFk << MatrixXf::Identity(4,4);
  rarmFk << MatrixXf::Identity(4,4);

  trans(current);

  for (int i=0;i<6;i++){
    llegFk = llegFk*lleg.trans.block<4,4>(4*i,0);
    rlegFk = rlegFk*rleg.trans.block<4,4>(4*i,0);
    larmFk = llegFk*larm.trans.block<4,4>(4*i,0);
    rarmFk = llegFk*rarm.trans.block<4,4>(4*i,0);
  }
  currentPose.segment<3>(0) = llegFk.block<3,1>(0,3);
  currentPose.segment<3>(6) = rlegFk.block<3,1>(0,3);
  currentPose.segment<3>(12) = larmFk.block<3,1>(0,3);
  currentPose.segment<3>(18) = rarmFk.block<3,1>(0,3);

return currentPose;
}

void trajectory (){
  // Define 4 goals;  
  VectorXf goal(24); //This probably needs to be 24
  goal << 0.2,-0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

  // Divide into steps
  VectorXf step(24);
  step.block<6,1>(0,0) << (0.002/step.block<3,1>(0,0).norm())*step.block<6,1>(0,0);
  step.block<6,1>(6,0) << (0.002/step.block<3,1>(6,0).norm())*step.block<6,1>(6,0);
  step.block<6,1>(12,0) << (0.002/step.block<3,1>(12,0).norm())*step.block<6,1>(12,0);
  step.block<6,1>(18,0) << (0.002/step.block<3,1>(18,0).norm())*step.block<6,1>(18,0);

  // Update array of current angles
  ros::spinOnce();

  // calculate current pose
  VectorXf currentPose(24);
  currentPose = fk(current);

  // Define the goal pose
  VectorXf goalPose(24);
  goalPose = currentPose + goal;

   while (abs(goalPose.norm()-currentPose.norm()) > abs(step.norm())){//this is wrong, do this better
    // calculate jacobian for each limb
     MatrixXf llegJ(6,6);
     MatrixXf rlegJ(6,6);
     MatrixXf larmJ(6,6);
     MatrixXf rarmJ(6,6);

    llegJ = lleg.jacobian();
    rlegJ = rleg.jacobian();
    larmJ =larm.jacobian();
    rarmJ = rarm.jacobian();

    // delQ = inverse jacobian multiplied by step.
    VectorXf delQ(24);
    MatrixXf temp(6,6);
    temp = llegJ.inverse();
    delQ.segment<6>(0) = temp*step.segment<6>(0);
    temp =  rlegJ.inverse();
    delQ.segment<6>(6) = temp*step.segment<6>(6);
    temp = larmJ.inverse();
    delQ.segment<6>(12) = temp*step.segment<6>(12);
    temp = rarmJ.inverse();
    delQ.segment<6>(18) = temp*step.segment<6>(18);    

    // cmd = current + delQ
    float temp1, temp2;
    for (int q = 4;q<28;q++){
      temp1 = current[q];
      temp2 = delQ[q-4];
      jointcommands.position[q] = temp1+temp2;
    }
//std::cout << "FUCK" << std::endl; 
    // publish commands
    pub_joint_commands_.publish(jointcommands);

    // get current angles
    ros::spinOnce();

    // calculate current poses
    currentPose = fk(current);

  }
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
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, SetJointStates,
    ros::VoidPtr(), rosnode->getCallbackQueue());
trajectory();

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

 // std::cout << sensor_msgs::JointState.position << std::endl;

  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);

 // ros::spin();

  return 0;
}    //pub_joint_commands_.publish(jointcommands);
