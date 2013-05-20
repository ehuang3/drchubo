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

// standard stuff
#include <math.h>
#include <iostream>

// ROS stuff
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <atlas_msgs/AtlasCommand.h>

// Boost stuff
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

// vrc stuff
#include <atlas/atlas_kinematics.h>
#include <utils/math_utils.h>
#include <utils/data_paths.h>

// DART stuff
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <dynamics/SkeletonDynamics.h>
#include <planning/RRT.h>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>
#include <planning/PathFollowingTrajectory.h>

ros::Publisher pub_joint_commands_;
atlas_msgs::AtlasCommand jointcommands;

Eigen::Vector6d angles;
atlas::atlas_kinematics_t AK;


std::map<int, std::string> ros_to_name;
std::map<int, int> jointmap_ros_to_dart;
std::map<int, int> jointmap_dart_to_ros;

simulation::World* mWorld;
dynamics::SkeletonDynamics* atlasSkel;

std::vector<int> left_arm;
std::vector<int> right_arm;
std::vector<int> left_leg;
std::vector<int> right_leg;

planning::Trajectory* left_arm_traj;
planning::Trajectory* right_arm_traj;
planning::Trajectory* left_leg_traj;
planning::Trajectory* right_leg_traj;

double left_arm_start;
double right_arm_start;
double left_leg_start;
double right_leg_start;

int findNamedDof(std::string name) {
    for (int i = 0; i < atlasSkel->getNumDofs(); i++) {
        if (name.compare(atlasSkel->getDof(i)->getName()) == 0) {
            return i;
        }
    }
    return -1;
}

std::vector<int> initializeLimb(size_t nnames, const std::string* names) {
    std::vector<int> result;
    result.resize(nnames);
    for(size_t i = 0; i < nnames; i++) {
        result[i] = atlasSkel->getJoint(names[i].c_str())->getFirstDofIndex();
    }
    return result;
}

planning::Trajectory* CreatePlan(const Eigen::VectorXd& current_state,
                                 const Eigen::VectorXd& limb_goal,
                                 const std::vector<int>& dofs) {

    Eigen::VectorXd init = Eigen::VectorXd::Zero(dofs.size());
    for (int i = 0; i < init.size(); i++) { init[i] = current_state[left_arm[i]]; }
    std::list<Eigen::VectorXd> path;
    planning::PathPlanner<planning::RRT> pathPlanner(*mWorld, true, true, .1, 1e6, 0.3);
    if(!pathPlanner.planPath(atlasSkel, dofs, init, limb_goal, path)) {
        std::cout << "could not find path" << std::endl;
        return NULL;
    }
    else {
        std::cout << "found a path" << std::endl;
        planning::PathShortener pathShortener(mWorld, atlasSkel, dofs);
        pathShortener.shortenPath(path);

        // Convert path into time-parameterized trajectory satisfying acceleration and velocity constraints
        const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(dofs.size());
        const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(dofs.size());
        planning::Trajectory* trajectory = new planning::PathFollowingTrajectory(path, maxVelocity, maxAcceleration);
        std::cout << "-- Trajectory duration: " << trajectory->getDuration() << std::endl;
        return trajectory;
    }
}

void executeTrajectory(double startTime, const std::vector<int>& limb, planning::Trajectory* traj, atlas_msgs::AtlasCommand& cmd) {
    double t = ros::Time::now().toSec() - startTime;
    if (t > 0.0 && t < traj->getDuration()) {
        for (unsigned int i = 0; i < limb.size(); i++) {
            std::cout << traj->getPosition(t)[i] << ", ";
            cmd.position[jointmap_dart_to_ros[limb[i]]] = traj->getPosition(t)[i];
        }
        std::cout << std::endl;
    }
}

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
    static ros::Time startTime = ros::Time::now();

    Eigen::VectorXd current_state = Eigen::VectorXd::Zero(atlasSkel->getNumDofs());
    for (unsigned int i = 0; i < _js->position.size(); i++)
        current_state[jointmap_ros_to_dart[i]] = _js->position[i];
    
    // left arm
    if (left_arm_traj == NULL) {
        Eigen::VectorXd la_goal = Eigen::VectorXd::Ones(left_arm.size());
        left_arm_traj = CreatePlan(current_state, la_goal, left_arm);
        left_arm_start = ros::Time::now().toSec();
    }
    else {
        executeTrajectory(left_arm_start, left_arm, left_arm_traj, jointcommands);
    }

    jointcommands.header.stamp = _js->header.stamp;
    pub_joint_commands_.publish(jointcommands);
}

int main(int argc, char** argv)
{
    std::cout << "-----DART init-----" << std::endl;
    DartLoader dart_loader;
    mWorld = dart_loader.parseWorld(VRC_DATA_PATH "models/atlas/atlas_world.urdf");
    atlasSkel = mWorld->getSkeleton("atlas");
    AK.init(atlasSkel);

    const std::string tla[] = {"l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx"};
    const std::string tra[] = {"r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"};
    const std::string tll[] = {"l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax"};
    const std::string trl[] = {"r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax"};
    left_arm = initializeLimb(6, tla);
    right_arm = initializeLimb(6, tra);
    left_leg = initializeLimb(6, tll);
    right_leg = initializeLimb(6, trl);

    left_arm_traj = NULL;
    right_arm_traj = NULL;
    left_leg_traj = NULL;
    right_leg_traj = NULL;
    std::cout << "-----DART init done-----" << std::endl;

    // Begin ROS init code

    std::cout << "-----ROS init start-----" << std::endl;
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
    ros_to_name = std::map<int, std::string>();
    ros_to_name[0] = "atlas::back_lbz";
    ros_to_name[1] = "atlas::back_mby";
    ros_to_name[2] = "atlas::back_ubx";
    ros_to_name[3] = "atlas::neck_ay";
    ros_to_name[4] = "atlas::l_leg_uhz";
    ros_to_name[5] = "atlas::l_leg_mhx";
    ros_to_name[6] = "atlas::l_leg_lhy";
    ros_to_name[7] = "atlas::l_leg_kny";
    ros_to_name[8] = "atlas::l_leg_uay";
    ros_to_name[9] = "atlas::l_leg_lax";
    ros_to_name[10] = "atlas::r_leg_uhz";
    ros_to_name[11] = "atlas::r_leg_mhx";
    ros_to_name[12] = "atlas::r_leg_lhy";
    ros_to_name[13] = "atlas::r_leg_kny";
    ros_to_name[14] = "atlas::r_leg_uay";
    ros_to_name[15] = "atlas::r_leg_lax";
    ros_to_name[16] = "atlas::l_arm_usy";
    ros_to_name[17] = "atlas::l_arm_shx";
    ros_to_name[18] = "atlas::l_arm_ely";
    ros_to_name[19] = "atlas::l_arm_elx";
    ros_to_name[20] = "atlas::l_arm_uwy";
    ros_to_name[21] = "atlas::l_arm_mwx";
    ros_to_name[22] = "atlas::r_arm_usy";
    ros_to_name[23] = "atlas::r_arm_shx";
    ros_to_name[24] = "atlas::r_arm_ely";
    ros_to_name[25] = "atlas::r_arm_elx";
    ros_to_name[26] = "atlas::r_arm_uwy";
    ros_to_name[27] = "atlas::r_arm_mwx";

    
    unsigned int n = ros_to_name.size();
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

    for (unsigned int i = 0; i < n; i++)
    {
        std::vector<std::string> pieces;
        boost::split(pieces, ros_to_name[i], boost::is_any_of(":"));
        double temp;

        int i_dart = findNamedDof(pieces[2]);
        assert(i_dart != -1);
        jointmap_ros_to_dart[i] = i_dart;
        jointmap_dart_to_ros[i_dart] = i;

        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p", temp);
        jointcommands.kp_position[i] = temp;

        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i", temp);
        jointcommands.ki_position[i] = temp;

        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d", temp);
        jointcommands.kd_position[i] = temp;

        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", temp);
        jointcommands.i_effort_min[i] = temp;
        jointcommands.i_effort_min[i] = -jointcommands.i_effort_min[i];
        
        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", temp);
        jointcommands.i_effort_max[i] = temp;

        jointcommands.velocity[i]     = 0;
        jointcommands.effort[i]       = 0;
        jointcommands.kp_velocity[i]  = 0;
        jointcommands.k_effort[i]     = 255;
    }

    pub_joint_commands_ =
        rosnode->advertise<atlas_msgs::AtlasCommand>(
            "/atlas/atlas_command", 1, true);

    // ros topic subscribtions
    ros::SubscribeOptions jointStatesSo =
        ros::SubscribeOptions::create<sensor_msgs::JointState>(
            "/atlas/joint_states", 1, SetJointStates,
            ros::VoidPtr(), rosnode->getCallbackQueue());
    jointStatesSo.transport_hints = ros::TransportHints().unreliable();
    ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);
    std::cout << "-----ROS init done-----" << std::endl;

    ros::spin();
  
    return 0;
}
