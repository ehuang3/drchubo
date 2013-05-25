//###########################################################
//###########################################################
//#### includes
//###########################################################
//###########################################################

// standard stuff
#include <math.h>
#include <iostream>

// ROS stuff
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <atlas_msgs/AtlasCommand.h>
#include <sensor_msgs/Joy.h>

// Boost stuff
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

// vrc stuff
#include <atlas/atlas_kinematics.h>
#include <utils/math_utils.h>
#include <utils/data_paths.h>
#include <atlas/atlas_state.h>
#include <atlas/atlas_jacobian.h>
#include "spnav_teleop.h"

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
#include <amino.h>

//###########################################################
//###########################################################
//#### variable declarations
//###########################################################
//###########################################################

simulation::World* mWorld;

ros::NodeHandle* rosnode;

dynamics::SkeletonDynamics* atlasSkel;
atlas::atlas_kinematics_t atlasKin;
atlas::atlas_state_rosified atlasStateTarget;
atlas::atlas_state_rosified atlasStateCurrent;
atlas::atlas_jacobian_t atlasJac;

ros::Publisher topic_pub_joint_commands;
atlas_msgs::AtlasCommand jointCommand;

//###########################################################
//###########################################################
//#### badly-written classes
//###########################################################
//###########################################################

namespace atlas {
    void atlas_state_rosified::fill_joint_command(atlas_msgs::AtlasCommand* jc,
                                                  ros::NodeHandle* rosnode)
    {
        unsigned int n_joints = g_r2s.size();
        
        jc->position.resize(n_joints);
        jc->velocity.resize(n_joints);
        jc->effort.resize(n_joints);
        jc->k_effort.resize(n_joints);
        jc->kp_position.resize(n_joints);
        jc->ki_position.resize(n_joints);
        jc->kd_position.resize(n_joints);
        jc->kp_velocity.resize(n_joints);
        jc->i_effort_min.resize(n_joints);
        jc->i_effort_max.resize(n_joints);

        for(unsigned int i = 0; i < n_joints; i++) {
            std::vector<std::string> pieces;
            boost::split(pieces, g_r2s[i], boost::is_any_of(":"));
            std::string ros_joint_name = pieces[2];
            double temp;
            
            rosnode->getParam("atlas_controller/gains/" + ros_joint_name + "/p", temp);
            jc->kp_position[i] = temp;

            rosnode->getParam("atlas_controller/gains/" + ros_joint_name + "/i", temp);
            jc->ki_position[i] = temp;

            rosnode->getParam("atlas_controller/gains/" + ros_joint_name + "/d", temp);
            jc->kd_position[i] = temp;

            rosnode->getParam("atlas_controller/gains/" + ros_joint_name + "/i_clamp", temp);
            jc->i_effort_min[i] = temp;
            jc->i_effort_min[i] = -temp;
        
            rosnode->getParam("atlas_controller/gains/" + ros_joint_name + "/i_clamp", temp);
            jc->i_effort_max[i] = temp;

            jc->velocity[i]     = 0;
            jc->effort[i]       = 0;
            jc->kp_velocity[i]  = 0;
            jc->k_effort[i]     = 255;
        }
    }
}

//###########################################################
//###########################################################
//#### initialization stuff
//###########################################################
//###########################################################

void topic_sub_joystick_handler(const sensor_msgs::Joy::ConstPtr& _j) {
    static std::vector<int> lastButtons = _j->buttons;
    static std::vector<float> lastAxes = _j->axes;
    static bool targetPoseInited = false;

    static ros::Time lastTime = ros::Time::now();
    ros::Time currentTime = ros::Time::now();
    ros::Duration dT = currentTime - lastTime;

    std::cout << _j->buttons[0] << ", " << !lastButtons[0] << std::endl;

    if (_j->buttons[0] && !lastButtons[0]) {
        targetPoseInited = true;
        std::cout << "Resetting target pose" << std::endl;
        Eigen::VectorXd temp;
        atlasStateCurrent.get_dart_pose(temp);
        atlasStateTarget.set_dart_pose(temp);
    }
    if (!targetPoseInited) { return; }

    Eigen::VectorXd movement(6); // the desired end effector velocity in worldspace
    for (unsigned int i = 0; i < 6; i++)
        movement[i] = .00005 * _j->axes[i] / dT.toSec();

    std::vector<int> desired_dofs; // which dofs to run the jacobian on
    Eigen::MatrixXd J;             // the forward jacobian
    kinematics::BodyNode* end_effector; // the bodynode on the end of our limb
    Eigen::VectorXd dofs;               // the configuration of the limb in jointspace
    Eigen::VectorXd qdot;               // the necessary jointspace velocity of the limb

    // compute the jacobian
    end_effector = atlasSkel->getNode("l_hand");
    atlasStateTarget.get_manip_indexes(desired_dofs, robot::LIMB_L_ARM);
    atlasJac.manip_jacobian(J, desired_dofs, end_effector, atlasStateTarget);

    // use the jacobian to get the jointspace velocity
    qdot = Eigen::VectorXd::Zero(J.cols());
    aa_la_dls(J.rows(), J.cols(), 0.1, J.data(), movement.data(), qdot.data());

    // use the jointspace velocity to update the target position
    dofs.resize(desired_dofs.size());
    atlasStateTarget.get_dofs(dofs, desired_dofs);
    atlasStateTarget.set_dofs(dofs + qdot, desired_dofs);

    // send the new target position to ROS
    Eigen::VectorXd rospose;
    atlasStateTarget.get_ros_pose(rospose);
    for(unsigned int i = 0; i < rospose.size(); i++) {
        jointCommand.position[i] = rospose[i];
    }
    
    jointCommand.header.stamp = ros::Time::now();
    topic_pub_joint_commands.publish(jointCommand);

    lastTime = currentTime;
    lastButtons = _j->buttons;
    lastAxes = _j->axes;
}

//###########################################################
//###########################################################
//#### ROS topic handlers
//###########################################################
//###########################################################

void topic_sub_joint_states_handler(const sensor_msgs::JointState::ConstPtr &_js) {
    static ros::Time startTime = ros::Time::now();

    Eigen::VectorXd r_pose(_js->position.size());
    for (unsigned int i = 0; i < _js->position.size(); i++)
        r_pose[i] = _js->position[i];
    atlasStateCurrent.set_ros_pose(r_pose);
}

//###########################################################
//###########################################################
//#### main
//###########################################################
//###########################################################


int main(int argc, char** argv) {
    //###########################################################
    //#### DART initialization
    DartLoader dart_loader;
    mWorld = dart_loader.parseWorld(VRC_DATA_PATH "models/atlas/atlas_world.urdf");
    atlasSkel = mWorld->getSkeleton("atlas"); // grab pointer to robot
    atlasKin.init(atlasSkel);                 // use robot to init kinematics
    atlasStateTarget.init(atlasSkel);         // init states
    atlasStateCurrent.init(atlasSkel);        // init states
    atlasJac.init(atlasSkel);                 // init jacobians

    //###########################################################
    //#### ROS initialization

    // initialize our ros node and block until we connect to the ROS
    // master
    ros::init(argc, argv, "trajectory_executor");
    rosnode = new ros::NodeHandle();
    ros::Time last_ros_time_;
    bool wait = true;
    while (wait) {
        last_ros_time_ = ros::Time::now();
        if (last_ros_time_.toSec() > 0) wait = false;
    }

    // set up the joint command
    atlasStateCurrent.fill_joint_command(&jointCommand, rosnode);
    
    // set up publishing
    topic_pub_joint_commands = rosnode->advertise<atlas_msgs::AtlasCommand>(
        "/atlas/atlas_command", 1, true);

    // set up subscriptions
    ros::SubscribeOptions topic_sub_joint_states_opts = ros::SubscribeOptions::create<sensor_msgs::JointState>(
        "/atlas/joint_states", 1, topic_sub_joint_states_handler,
        ros::VoidPtr(), rosnode->getCallbackQueue());
    topic_sub_joint_states_opts.transport_hints = ros::TransportHints().unreliable();
    ros::Subscriber topic_sub_joint_states = rosnode->subscribe(topic_sub_joint_states_opts);

    ros::SubscribeOptions topic_sub_joystick_opts = ros::SubscribeOptions::create<sensor_msgs::Joy>(
        "joy", 1, topic_sub_joystick_handler,
        ros::VoidPtr(), rosnode->getCallbackQueue());
    topic_sub_joystick_opts.transport_hints = ros::TransportHints().unreliable();
    ros::Subscriber topic_sub_joystick = rosnode->subscribe(topic_sub_joystick_opts);    
    ros::spin();
    return 0;
}
