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
#include "teleop_demo.h"

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
#include <gui/teleop_gui.h>
#include <pthread.h>

// SPACENAV stuff
#include "spnav.h"

// Control stuff
#include <control/control.h>
#include <control/control_factory.h>
#include <control/arms.h>


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

pthread_t gui_thread;
gui::teleop_gui_t gui_window;
gui::params_t gui_params;

bool gazebo_sim; //< we are simulation in gazebo

teleop::spnav_t spnav;

control::control_data_t* c_data;
control::control_t* controller;

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
//#### World orientation math
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
//#### Main loop
//###########################################################
//###########################################################

void topic_sub_joystick_handler(const sensor_msgs::Joy::ConstPtr& _j) {
    //############################################################
    //### Update peripherals
    if(!spnav.sensor_update(_j))
        return;
    spnav.get_teleop_data(c_data);
    
    //############################################################
    //### Init states
    static bool targetPoseInited = false;
    static bool print_once = true;
    if (!targetPoseInited && print_once) {
        std::cout << "Please init target pose first - hit the left button on the spacenav base." << std::endl;
        print_once = false;
    }
    if (c_data->triggers[0]) {
        // 1. Set target pose to match current
        std::cout << "Setting target pose." << std::endl;
        Eigen::VectorXd temp;
        atlasStateCurrent.get_ros_pose(temp);
        atlasStateTarget.set_ros_pose(temp);

        // 2. Set world origins to feet
        move_origin_to_feet( atlasStateTarget );
        move_origin_to_feet( atlasStateCurrent );
        targetPoseInited = true;

        // 3. Set target com to current
        atlasSkel->setPose( atlasStateCurrent.dart_pose() );
        c_data->com = atlasSkel->getWorldCOM();

        // 4. Set last command to current
        c_data->last_command = atlasStateCurrent.ros_pose();

        // 5. Set manip xforms
        c_data->manip_xform[robot::MANIP_L_HAND] = atlasSkel->getNode(ROBOT_LEFT_HAND)->getWorldTransform();
        c_data->manip_xform[robot::MANIP_R_HAND] = atlasSkel->getNode(ROBOT_RIGHT_HAND)->getWorldTransform();
        c_data->manip_xform[robot::MANIP_L_FOOT] = atlasSkel->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
        c_data->manip_xform[robot::MANIP_R_FOOT] = atlasSkel->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();
    }
    if (!targetPoseInited)
        return;
    
    //############################################################
    //### Switch states
    if (c_data->triggers[1]) {
        // Do the state switch here...
    }
    
    //############################################################
    //### Run controller
    if (controller && c_data->sensor_ok && c_data->joystick_ok)
        controller->run(atlasStateTarget, c_data);

    //############################################################
    //### Send joint commands
    Eigen::VectorXd rospose;
    atlasStateTarget.get_ros_pose(rospose);
    if (gazebo_sim) {
        for(unsigned int i = 0; i < rospose.size(); i++) {
            jointCommand.position[i] = rospose[i];
        }
        jointCommand.header.stamp = _j->header.stamp;
        topic_pub_joint_commands.publish(jointCommand);
        
        // Errors
        double motion_norm = (c_data->last_command - atlasStateTarget.ros_pose()).norm();
        std::cout << "joint delta norm = " << motion_norm << std::endl;
    
        double error_norm = (atlasStateCurrent.ros_pose() - atlasStateTarget.ros_pose()).norm();
        std::cout << "pid error norm = " << error_norm << std::endl;
    }
    c_data->last_command = rospose;

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
    //#### Controll initilization
    c_data = new control::control_data_t();
    c_data->robot = atlasSkel;
    c_data->current = &atlasStateCurrent;
    c_data->kin = &atlasKin;
    c_data->jac = &atlasJac;

    controller = control::get_factory("ARM_AJIK")->create();

    //###########################################################
    //#### GUI initialization
    c_data->manip_target = Eigen::Matrix4d::Identity();
    gui_params.goal = &c_data->manip_target;
    gui_params.current = &atlasStateCurrent;
    gui_params.target = &atlasStateTarget;
    gui_params.draw_limits = true;
    gui_window.gui_params = &gui_params;
    pthread_create(&gui_thread, NULL, gui::teleop_gui_t::start_routine, &gui_window);

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
    ros::SubscribeOptions topic_sub_joystick_opts = ros::SubscribeOptions::create<sensor_msgs::Joy>(
        "/spacenav/joy", 1, topic_sub_joystick_handler,
        ros::VoidPtr(), rosnode->getCallbackQueue());

    topic_sub_joystick_opts.transport_hints = ros::TransportHints().unreliable();
    ros::Subscriber topic_sub_joystick = rosnode->subscribe(topic_sub_joystick_opts);

    //###########################################################
    //#### Connect to simulation and start spinning
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
