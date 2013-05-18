/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>


#include <atlas/atlas_kinematics.h>
#include <utils/math_utils.h>
#include <utils/data_paths.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/Skeleton.h>
#include <dynamics/SkeletonDynamics.h>

using namespace std;
using namespace atlas;
using namespace kinematics;
using namespace Eigen;
using namespace dynamics;
using namespace simulation;

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;

Vector6d angles;
atlas_kinematics_t AK;

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
  static ros::Time startTime = ros::Time::now();
  {
    // for testing round trip time
    jointcommands.header.stamp = _js->header.stamp;

    double dt = (ros::Time::now() - startTime).toSec();
    double r = 0.25;
    double x = r * cos(dt);
    double z = r * sin(dt) + r;

    Matrix4d Tfoot;
    Tfoot.row(0) <<  0, 0, 1,  x    ;
    Tfoot.row(1) <<  0, 1, 0,  0    ;
    Tfoot.row(2) << -1, 0, 0, -0.846 + z;
    Tfoot.row(3) <<  0, 0, 0,  1    ;

    Vector6d u;
    u << 0, 0, 0, 0, 0, 0;
    try {
      AK.legIK(Tfoot, true, u, angles);
    } catch (char const* msg) {
      cerr << msg << endl;
      return;
    }

    // assign sinusoidal joint angle targets
    for (unsigned int i = 0; i < jointcommands.name.size(); i++)
    {
      string name = jointcommands.name[i];
      jointcommands.position[i] = 0;
      if(name == "atlas::l_leg_uhz")
      {
        jointcommands.position[i] = angles[0];
      }
      if(name == "atlas::l_leg_mhx")
      {
        jointcommands.position[i] = angles[1];
      }
      if(name == "atlas::l_leg_lhy")
      {
        jointcommands.position[i] = angles[2];
      }
      if(name == "atlas::l_leg_kny")
      {
        jointcommands.position[i] = angles[3];
      }
      if(name == "atlas::l_leg_uay")
      {
        jointcommands.position[i] = angles[4];
      }
      if(name == "atlas::l_leg_lax")
      {
        jointcommands.position[i] = angles[5];
      }
    }
      //jointcommands.position[i] = 3.2* sin((ros::Time::now() - startTime).toSec());

    pub_joint_commands_.publish(jointcommands);
  }
}

int main(int argc, char** argv)
{
  cout << "-----DART init-----" << endl;
  DartLoader dart_loader;
  World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH "models/atlas/atlas_world.urdf");
  SkeletonDynamics *atlas = mWorld->getSkeleton("atlas");
  cout << endl << "-----done-----" << endl << endl;

  AK.init(atlas);

  // Begin ROS init code

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

  ros::spin();

  return 0;
}