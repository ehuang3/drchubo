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
#include <utils/robot_configs.h>
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
atlas::atlas_kinematics_t atlasKin;

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

    static ros::Time lastTime = _j->header.stamp;
    ros::Time currentTime = _j->header.stamp;
    ros::Duration dT = currentTime - lastTime;

    robot::LimbIndex targetLimb = robot::LIMB_L_ARM;
    std::vector<int> desired_dofs; // which dofs to run the jacobian on
    kinematics::BodyNode* end_effector; // the bodynode on the end of our limb
    atlasStateTarget.get_manip_indexes(desired_dofs, targetLimb);
    end_effector = atlasSkel->getNode("l_hand");

    if (_j->buttons[0] && !lastButtons[0]) {
        std::cout << "Setting target pose." << std::endl;
        Eigen::VectorXd temp;
        atlasStateCurrent.get_ros_pose(temp);
        atlasStateTarget.set_ros_pose(temp);
        targetPoseInited = true;
    }
    
    if (!targetPoseInited) {
        std::cout << "Please init target pose first - hit the left button on the spacenav base." << std::endl;
    }
    if (targetPoseInited && !_j->buttons[0]) {
        double movespeed = .00005;
        Eigen::VectorXd movement(6); // the desired end effector velocity in worldspace
        movement[0] = movespeed * _j->axes[1] / dT.toSec(); // translate +x
        movement[1] = movespeed * _j->axes[0] / dT.toSec(); // translate +y
        movement[2] = movespeed * _j->axes[2] / dT.toSec(); // translate +z
        movement[3] = movespeed * _j->axes[4] / dT.toSec(); // rotate + around x
        movement[4] = movespeed * _j->axes[3] / dT.toSec(); // rotate + around y
        movement[5] = movespeed * _j->axes[5] / dT.toSec(); // rotate + around z
        // Sanity check
        bool invalid_input = dT.toSec() == 0;
        for(int i=0; i < 6; i++) {
            invalid_input |= isnan(movement[0]);
        }
        // Convert to vectors
        Vector3d off = Vector3d(movement[0], movement[1], movement[2]);
        Vector3d rot = Vector3d(movement[3], movement[4], movement[5]);
        double rot_angle = rot.norm();
        Vector3d rot_axis = rot.normalize();

        // Put world origin between atlas's feet (for meaningful comik)
        Eigen::Isometry3d Twlf, Twrf, Twb; //< xform world to: left foot, right foot, body
        Eigen::Isometry3d Two; //< world to new origin
        Twlf = atlasSkel->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
        Twrf = atlasSkel->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();
        atlasStateCurrent.get_body(Twb); //< world position of body center (pelvis)
        // Average feet locations
        Two.linear() = (Twlf.rotation() * Twrf.rotation()).sqrt();
        Two.translation() = (Twlf.translation() + Twrf.translation())/2;
        Eigen::Isometry3d Tolf = Two.inverse() * Twlf; //< origin to left foot
        Two.translation() = Vector3d::Zero();
        Twb = Two * Tolf * Twlf.inverse() * Twb; //< current location of body (assuming feet on ground)
        
        // Transform body frame by joy stick amount
        Twb.translation() += Vector3d(movement[0]);
        Twb.rotate(AngleAxisd(rot_angle, rot_axis));

        // Position body and keep COM in between feet
        Vector3d com = Vector3d::Zero();
        atlasStateCurrent.set_body(Twb);
        atlasStateTarget.set_body(Twb);
        // COM height will follow joystick
        com(2) += off(2);

        // COM ik to 
        bool ok = atlasKin.com_ik(

            if(!ok) {
                // freak out
            }
        
        // for (unsigned int i = 0; i < 6; i++)
        //     movement[i] = .0001 * _j->axes[i] / dT.toSec();
        // for (unsigned int i = 0; i < 6; i++)
        //     if (i != 0) movement[i] = 0.0;

        Eigen::MatrixXd J;             // the forward jacobian
        Eigen::VectorXd dofs;               // the configuration of the limb in jointspace
        Eigen::VectorXd qdot;               // the necessary jointspace velocity of the limb

        // compute the jacobian
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
    
        jointCommand.header.stamp = _j->header.stamp;
        topic_pub_joint_commands.publish(jointCommand);
    }

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
    atlasKin.init(atlasSkel);                 // init kinematics

    //###########################################################
    //#### ROS initialization

    // initialize our ros node and block until we connect to the ROS
    // master
    ros::init(argc, argv, "spacenav_teloperator");
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
