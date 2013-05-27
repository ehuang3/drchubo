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
#include "spnav_com.h"

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

Eigen::Vector3d com;
Eigen::VectorXd lastCommand;

Eigen::Vector6d joy_thresh; //< static spacenav readings have noise
int max_thresh_iters;
int thresh_iters;

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
//#### com math
//###########################################################
//###########################################################
// Calculates the orientation of atlas's body given a foot position
void move_origin_to_feet(atlas::atlas_state_t& atlasState)
{
    // 1. Get skeleton for computation
    kinematics::Skeleton *atlasSkel = atlasState.robot();
    atlasSkel->setPose( atlasState.dart_pose() );
    
    // 2. Get left foot, right foot, and body xforms from skeleton
    Eigen::Isometry3d Twlf, Twrf, Twb; //< xform world to: left foot, right foot, body
    Eigen::Isometry3d Two; //< world to new origin
    Twlf = atlasSkel->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
    Twrf = atlasSkel->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();
    Twb = atlasSkel->getNode(ROBOT_BODY)->getWorldTransform();

    // 3. Orient atlas's body around his left foot.
    Eigen::Isometry3d Tlfb = Twlf.inverse() * Twb;
    atlasState.set_body(Tlfb); // world origin at left foot
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
    
    // Triggers on a left button click
    if (_j->buttons[0] && !lastButtons[0]) {
        // 0. Set target pose to match current
        std::cout << "Setting target pose." << std::endl;
        Eigen::VectorXd temp;
        atlasStateCurrent.get_ros_pose(temp);
        atlasStateTarget.set_ros_pose(temp);
        targetPoseInited = true;
        // 1. Set target com to be current
        move_origin_to_feet( atlasStateCurrent );
        atlasSkel->setPose( atlasStateCurrent.dart_pose() );
        com = atlasSkel->getWorldCOM();
        std::cout << "Starting com = " << com.transpose() << std::endl;
        // 2. Display joystick thresholds and normal
        std::cout << "Joystick thresholds = " << joy_thresh.transpose() << std::endl;
        std::cout << "Joystick normal = " << joy_thresh.norm() << std::endl;
        // 3. Init last command as current state
        atlasStateCurrent.get_ros_pose( lastCommand );
    }
    if (!targetPoseInited) {
        std::cout << "Please init target pose first - hit the left button on the spacenav base." << std::endl;
    }
    if (thresh_iters++ < max_thresh_iters) {
        // 0. Update error. Don't touch the joystick!
        for(int i=0; i < 6; i++)
            if(fabs(_j->axes[i]) > joy_thresh(i))
                joy_thresh(i) = fabs(_j->axes[i]);
        if (thresh_iters == max_thresh_iters)
            std::cout << "Joystick thresholds = " << joy_thresh.transpose() << std::endl;
    }
    for (; targetPoseInited && !_j->buttons[0] ;) {
        // 0. Threshold the joystick values using norm
        double thresh = 2*joy_thresh.norm(); // add some slack (on release, joysick will not bounce back to zero)
        Eigen::VectorXd true_axes(6);
        Eigen::VectorXd joy_axes(6);
        joy_axes.setZero();
        bool allBelow = true;
        for(int i=0; i < 6; i++) {
            true_axes(i) = _j->axes[i];
            if( fabs(_j->axes[i]) <= thresh )
                continue;
            joy_axes[i] = _j->axes[i];
            allBelow = false;
        }
        if (allBelow) {
            break;
        }
        
        // 2. Convert to move speeds 
        double movespeed = .00005;
        Eigen::VectorXd movement(6); // the desired end effector velocity in worldspace
        movement[0] = movespeed * joy_axes[1] / dT.toSec(); // translate +x
        movement[1] = movespeed * joy_axes[0] / dT.toSec(); // translate +y
        movement[2] = movespeed * joy_axes[2] / dT.toSec(); // translate +z
        movement[3] = movespeed * joy_axes[4] / dT.toSec(); // rotate + around x
        movement[4] = movespeed * joy_axes[3] / dT.toSec(); // rotate + around y
        movement[5] = movespeed * joy_axes[5] / dT.toSec(); // rotate + around z
        // -48. Sanity check
        bool invalid_input = dT.toSec() == 0;
        for(int i=0; i < 6; i++) {
            invalid_input |= isnan(movement[0]);
            invalid_input |= isinf(movement[0]);
        }
        if (invalid_input) {
            std::cout << "invalid movement = " << movement.transpose() << std::endl;
            break;
        }
        
        // 0. Convert to movements to vectors
        Eigen::Vector3d off = Eigen::Vector3d(movement[0], movement[1], movement[2]);
        Eigen::Vector3d rot = Eigen::Vector3d(movement[3], movement[4], movement[5]);

        // 0.5. Update target to match current (MAY CAUSE SUPER PROBLEMS)
        // atlasStateTarget.set_dart_pose( atlasStateCurrent.dart_pose() );

        // 1. Maintain a fixed target state. Send those joint commands to Atlas. Make observations.
        atlasSkel->setPose( atlasStateTarget.dart_pose() );

        // 2. Orient Atlas about his feet
        move_origin_to_feet( atlasStateTarget );
        atlasSkel->setPose(atlasStateTarget.dart_pose()); //< save into skel so we can compute with it

        // 4. Grab the target state COM and offset by spacenav (might shake atlas around..)
        com = atlasSkel->getWorldCOM();
        com += off;
        std::cout << "desired com = " << com.transpose() << std::endl;

        // 5. Do IK to target state COM. (Should give the exact same joint angles)
        Eigen::Isometry3d Twlf, Twrf;
        Eigen::Isometry3d end_effectors[robot::NUM_MANIPULATORS];
        Twlf = atlasSkel->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
        Twrf = atlasSkel->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();
        end_effectors[robot::MANIP_L_FOOT] = Twlf;
        end_effectors[robot::MANIP_R_FOOT] = Twrf;
        
        robot::IK_Mode mode[robot::NUM_MANIPULATORS];
        mode[0] = robot::IK_MODE_SUPPORT;
        mode[1] = robot::IK_MODE_WORLD;
        mode[2] = robot::IK_MODE_FREE;
        mode[3] = robot::IK_MODE_FREE;

        bool ok = atlasKin.com_ik(com, end_effectors, mode, atlasStateTarget);

        // 6.1 Freak out
        if(!ok) {
            std::cerr << "com ik failed\n" << std::endl;
            break;
        }
        
        // 6.2 joint delta norm
        double omega_norm = (lastCommand - atlasStateTarget.ros_pose()).norm();
        std::cout << "joint delta norm = " << omega_norm << std::endl;
        
        // 7. Send commands over ROS
        Eigen::VectorXd rospose;
        atlasStateTarget.get_ros_pose(rospose);
        for(unsigned int i = 0; i < rospose.size(); i++) {
            jointCommand.position[i] = rospose[i];
        }
            
        jointCommand.header.stamp = _j->header.stamp;
        topic_pub_joint_commands.publish(jointCommand);
        lastCommand = rospose;
        
        // 99. We're in a for loop
        break;
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
    //#### Joystick initialization
    joy_thresh = Eigen::Vector6d::Zero();
    max_thresh_iters = 100;
    thresh_iters = 0;

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
        "/spacenav/joy", 1, topic_sub_joystick_handler,
        ros::VoidPtr(), rosnode->getCallbackQueue());
    topic_sub_joystick_opts.transport_hints = ros::TransportHints().unreliable();
    ros::Subscriber topic_sub_joystick = rosnode->subscribe(topic_sub_joystick_opts);    
    ros::spin();
    return 0;
}
