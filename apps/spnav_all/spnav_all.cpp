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
#include "spnav_all.h"

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

// GUI stuff
#include "MyWindow.h"
#include <pthread.h>

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

pthread_t gui_thread;
MyWindow gui_window;

int teleopMode;
Eigen::Isometry3d manip_xforms[robot::NUM_MANIPULATORS];

bool gazebo_sim; //< we are simulation in gazebo

Eigen::Isometry3d xformTarget; //< Target transformation of teleoperation

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
        
        double gain_scale = 2;

        for(unsigned int i = 0; i < n_joints; i++) {
            std::vector<std::string> pieces;
            boost::split(pieces, g_r2s[i], boost::is_any_of(":"));
            std::string ros_joint_name = pieces[2];
            double temp;
            
            rosnode->getParam("atlas_controller/gains/" + ros_joint_name + "/p", temp);
            jc->kp_position[i] = gain_scale * temp;

            rosnode->getParam("atlas_controller/gains/" + ros_joint_name + "/i", temp);
            jc->ki_position[i] = gain_scale * temp;

            rosnode->getParam("atlas_controller/gains/" + ros_joint_name + "/d", temp);
            jc->kd_position[i] = gain_scale * temp;

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

    //############################################################
    //### Initialize states
    //############################################################
    static bool targetPoseInited = false;

    // 1. Triggers on a left button click
    if (_j->buttons[0] && !lastButtons[0]) {
        // 1. Set target pose to match current
        std::cout << "Setting target pose." << std::endl;
        Eigen::VectorXd temp;
        atlasStateCurrent.get_ros_pose(temp);
        atlasStateTarget.set_ros_pose(temp);
        move_origin_to_feet( atlasStateTarget );
        move_origin_to_feet( atlasStateCurrent );
        targetPoseInited = true;
        // 1. Set target com to be current
        atlasSkel->setPose( atlasStateCurrent.dart_pose() );
        com = atlasSkel->getWorldCOM();
        std::cout << "Starting com = " << com.transpose() << std::endl;
        // 2. Display joystick thresholds and normal
        std::cout << "Joystick thresholds = " << joy_thresh.transpose() << std::endl;
        std::cout << "Joystick normal = " << joy_thresh.norm() << std::endl;
        // 3. Init last command as current state
        atlasStateCurrent.get_ros_pose( lastCommand );

        // 4. Init manip xforms
        atlasSkel->setPose( atlasStateCurrent.dart_pose() );
        manip_xforms[robot::MANIP_L_HAND] = atlasSkel->getNode(ROBOT_LEFT_HAND)->getWorldTransform();
        manip_xforms[robot::MANIP_R_HAND] = atlasSkel->getNode(ROBOT_RIGHT_HAND)->getWorldTransform();
        manip_xforms[robot::MANIP_L_FOOT] = atlasSkel->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
        manip_xforms[robot::MANIP_R_FOOT] = atlasSkel->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();
    }
    static bool print_once = true;
    if (!targetPoseInited && print_once) {
        std::cout << "Please init target pose first - hit the left button on the spacenav base." << std::endl;
        print_once = false;
    }
    // 2. Update error. Don't touch the joystick!
    if (thresh_iters++ < max_thresh_iters) {
        for(int i=0; i < 6; i++)
            if(fabs(_j->axes[i]) > joy_thresh(i))
                joy_thresh(i) = fabs(_j->axes[i]);
        if (thresh_iters == max_thresh_iters)
            std::cout << "Joystick thresholds = " << joy_thresh.transpose() << std::endl;
    }
    //############################################################
    //### State switching
    //############################################################
    robot::LimbIndex targetLimb = robot::LIMB_L_ARM;
    std::vector<int> desired_dofs; // which dofs to run the jacobian on
    kinematics::BodyNode* end_effector; // the bodynode on the end of our limb
    bool left = true;
    
    // 1. Trigger switch on right button click
    if (_j->buttons[1] && !lastButtons[1]) {
        teleopMode = (teleopMode + 1) % NUM_TELEOP_MODES;
        switch (teleopMode) {
        case TELEOP_COM: std::cout << "Teleop Mode: COM\n" << std::endl; break;
        case TELEOP_LEFT_ARM_JACOBIAN: std::cout << "Teleop Mode: Left Arm Jacobian\n" << std::endl; break;
        case TELEOP_RIGHT_ARM_JACOBIAN: std::cout << "Teleop Mode: Right Arm Jacobian\n" << std::endl; break;
        case TELEOP_LEFT_ARM_ANALYTIC: std::cout << "Teleop Mode: Left Arm Analytic\n" << std::endl; break;
        case TELEOP_RIGHT_ARM_ANALYTIC: std::cout << "Teleop Mode: Right Arm Analytic\n" << std::endl; break;
        default: break;
        }
    }
    // 2. Setup parameters
    switch (teleopMode) {
    case TELEOP_COM: break;
    case TELEOP_LEFT_ARM_ANALYTIC:
    case TELEOP_LEFT_ARM_JACOBIAN:
        left = true;
        targetLimb = robot::LIMB_L_ARM;
        atlasStateTarget.get_manip_indexes(desired_dofs, targetLimb);
        end_effector = atlasSkel->getNode(ROBOT_LEFT_HAND);
        break;
    case TELEOP_RIGHT_ARM_ANALYTIC:
    case TELEOP_RIGHT_ARM_JACOBIAN: 
        left = false;
        targetLimb = robot::LIMB_R_ARM;
        atlasStateTarget.get_manip_indexes(desired_dofs, targetLimb);
        end_effector = atlasSkel->getNode(ROBOT_RIGHT_HAND);
        break;
    default: break;
    }
    //############################################################
    //### Joystick input
    //############################################################ 
    static ros::Time lastTime = _j->header.stamp;
    ros::Time currentTime = _j->header.stamp;
    ros::Duration dT = currentTime - lastTime;

    // 1. Threshold the joystick values using norm
    double thresh = 2*joy_thresh.norm(); // add some slack (on release, joysick will not bounce back to zero)
    int max_scale = 2; // scale thresh to get max input caps
    Eigen::VectorXd true_axes(6);
    Eigen::VectorXd joy_axes(6);
    joy_axes.setZero();
    bool allBelow = true;
    for(int i=0; i < 6; i++) {
        true_axes(i) = _j->axes[i];
        if( fabs(_j->axes[i]) <= thresh )
            continue;
        joy_axes[i] = _j->axes[i];
        if( fabs(joy_axes[i]) > max_scale * thresh )
            joy_axes[i] *= max_scale*thresh / fabs(joy_axes[i]);
        allBelow = false;
    }
    // 2. Convert to move speeds 
    double movespeed = .00005;
    Eigen::VectorXd movement(6); // the desired end effector velocity in worldspace
    movement[0] = movespeed * joy_axes[0] / dT.toSec(); // translate +x
    movement[1] = movespeed * joy_axes[1] / dT.toSec(); // translate +y
    movement[2] = movespeed * joy_axes[2] / dT.toSec(); // translate +z
    movement[3] = movespeed * joy_axes[4] / dT.toSec(); // rotate + around x
    movement[4] = movespeed * joy_axes[3] / dT.toSec(); // rotate + around y
    movement[5] = movespeed * joy_axes[5] / dT.toSec(); // rotate + around z
    // 3. Sanity check
    bool invalid_input = dT.toSec() == 0;
    for(int i=0; i < 6; i++) {
        invalid_input |= isnan(movement[0]);
        invalid_input |= isinf(movement[0]);
    }
    if (invalid_input) {
        std::cout << "invalid movement = " << movement.transpose() << std::endl;
    }
    // 4. Convert to movements to vectors
    Eigen::Vector3d movement_offset = Eigen::Vector3d(movement[0], movement[1], movement[2]);
    Eigen::Vector3d movement_rotation = Eigen::Vector3d(movement[3], movement[4], movement[5]);

    //############################################################
    //### Teleop control
    //############################################################
    for (; targetPoseInited && !_j->buttons[0] ;) { //< so I can break... dance
        // 0. Breaks
        if (allBelow || invalid_input)
            break;

        // 1. Maintain a fixed target state. Send those joint commands to Atlas. Make observations.
        atlasSkel->setPose( atlasStateTarget.dart_pose() );
        
        bool ok = true;
        switch (teleopMode) {
        case TELEOP_COM:
        {
            // 1. Save dofs
            Eigen::VectorXd dofs = atlasStateTarget.dart_pose();

            // 2. Orient Atlas about his feet
            move_origin_to_feet( atlasStateTarget );
            atlasSkel->setPose(atlasStateTarget.dart_pose()); //< save into skel so we can compute with it

            // 3. Grab the target state COM and offset by spacenav (might shake atlas around..)
            // com = atlasSkel->getWorldCOM();
            com += movement_offset;
            std::cout << "desired com = " << com.transpose() << std::endl;

            // 4. Do IK to target state COM. (Should give the exact same joint angles)
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

            ok = atlasKin.com_ik(com, end_effectors, mode, atlasStateTarget);

            // 5. Write xform target
            xformTarget = Eigen::Matrix4d::Identity();
            xformTarget.translation() += com;

            // 6. Do not move if com ik failed
            if(!ok)
                atlasStateTarget.set_dart_pose(dofs);

            break;
        }
        case TELEOP_LEFT_ARM_JACOBIAN:
        case TELEOP_RIGHT_ARM_JACOBIAN:
        {
            // 2. Swap rotation axes so they make sense for a teleoperator
            std::swap(movement[3], movement[4]);
            // 3. Jacobian control
            Eigen::MatrixXd J;  // the forward jacobian
            Eigen::VectorXd dofs; // the configuration of the limb in jointspace
            Eigen::VectorXd qdot; // the necessary jointspace velocity of the limb

            // compute the jacobian
            atlasJac.manip_jacobian(J, desired_dofs, end_effector, atlasStateTarget);

            // use the jacobian to get the jointspace velocity
            qdot = Eigen::VectorXd::Zero(J.cols());
            aa_la_dls(J.rows(), J.cols(), 0.1, J.data(), movement.data(), qdot.data());

            // use the jointspace velocity to update the target position
            dofs.resize(desired_dofs.size());
            atlasStateTarget.get_dofs(dofs, desired_dofs);
            atlasStateTarget.set_dofs(dofs + qdot, desired_dofs);
            break;
        }
        case TELEOP_LEFT_ARM_ANALYTIC:
        case TELEOP_RIGHT_ARM_ANALYTIC:
        {
            // 2. Swap rotation axes so they make sense for a teleoperator
            std::swap(joy_axes[3], joy_axes[4]);
            // 3.. Generate delta transform
            Eigen::Isometry3d Tdelta;
            Tdelta = Eigen::Matrix4d::Identity();
            Eigen::Vector3d rot_axis = joy_axes.block<3,1>(3,0);
            double rot_omega = 2*movement_rotation.norm(); //< 
            rot_axis = rot_axis.normalized();
            std::cout << "rot_axis = " << rot_axis.transpose() << std::endl;
            if(!isnan(rot_axis[0])) // it happens when rot_axis is all zeros (when we clamp)
                Tdelta.rotate(Eigen::AngleAxisd(rot_omega, rot_axis));
            // 4. Generate new hand transform
            Eigen::Isometry3d Twhand;
            Eigen::VectorXd dofs;
            Eigen::Vector6d q6;
            Twhand = manip_xforms[targetLimb];
            Twhand = Twhand * Tdelta;
            Twhand.translation() += movement_offset;
            // 5. Run IK
            ok = atlasKin.arm_jac_ik(Twhand, left, atlasStateTarget);
            // 6. Write xform target
            xformTarget = Twhand;
            manip_xforms[targetLimb] = Twhand;
            break;
        }
        default:
            break;
        }
        
        // 6.1 Freak out
        if(!ok) {
            std::cerr << "ik failed\n" << std::endl;
            break;
        }
        
        // 6.2 joint delta norm
        double motion_norm = (lastCommand - atlasStateTarget.ros_pose()).norm();
        std::cout << "joint delta norm = " << motion_norm << std::endl;
        
        double error_norm = (atlasStateCurrent.ros_pose() - atlasStateTarget.ros_pose()).norm();
        std::cout << "pid error norm = " << error_norm << std::endl;
        
        // 7. Send commands over ROS
        Eigen::VectorXd rospose;
        atlasStateTarget.get_ros_pose(rospose);
        if (gazebo_sim) {
            for(unsigned int i = 0; i < rospose.size(); i++) {
                jointCommand.position[i] = rospose[i];
            }
            jointCommand.header.stamp = _j->header.stamp;
            topic_pub_joint_commands.publish(jointCommand);
        }
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
    move_origin_to_feet(atlasStateCurrent);
}

//###########################################################
//###########################################################
//#### main
//###########################################################
//###########################################################


int main(int argc, char** argv) {
    //###########################################################
    //#### Command line arguments
    gazebo_sim = true;
    if (argc == 2) {
        std::string in(argv[1]);
        if(in == "--no-sim")
            gazebo_sim = false;
    }
    std::cout << "gazebo sim = " << gazebo_sim << std::endl;

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
    //#### GUI initialization
    xformTarget = Eigen::Matrix4d::Identity();
    gui_window.setCurrentState(&atlasStateCurrent);
    gui_window.setTargetState(&atlasStateTarget);
    gui_window.setTargetXform(&xformTarget);
    pthread_create(&gui_thread, NULL, MyWindow::start_routine, &gui_window);

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



    //###########################################################
    //#### Joystick initialization
    joy_thresh = Eigen::Vector6d::Zero();
    max_thresh_iters = 100;
    thresh_iters = 0;

    teleopMode = TELEOP_LEFT_ARM_ANALYTIC;

    ros::SubscribeOptions topic_sub_joystick_opts = ros::SubscribeOptions::create<sensor_msgs::Joy>(
        "/spacenav/joy", 1, topic_sub_joystick_handler,
        ros::VoidPtr(), rosnode->getCallbackQueue());
    topic_sub_joystick_opts.transport_hints = ros::TransportHints().unreliable();
    ros::Subscriber topic_sub_joystick = rosnode->subscribe(topic_sub_joystick_opts);

    //###########################################################
    //#### OK GO
    // set up publishing
    if(gazebo_sim) {
        // set up the joint command
        atlasStateCurrent.fill_joint_command(&jointCommand, rosnode);

        topic_pub_joint_commands = rosnode->advertise<atlas_msgs::AtlasCommand>(
            "/atlas/atlas_command", 1, true);

        // set up subscriptions
        ros::SubscribeOptions topic_sub_joint_states_opts = ros::SubscribeOptions::create<sensor_msgs::JointState>(
            "/atlas/joint_states", 1, topic_sub_joint_states_handler,
            ros::VoidPtr(), rosnode->getCallbackQueue());
        topic_sub_joint_states_opts.transport_hints = ros::TransportHints().unreliable();
        ros::Subscriber topic_sub_joint_states = rosnode->subscribe(topic_sub_joint_states_opts);

        ros::spin();
    } else {
        ros::spin();
    }
    return 0;
}
