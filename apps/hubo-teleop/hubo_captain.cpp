//############################################################
//# Includes
//############################################################
// standard stuff
#include <math.h>
#include <iostream>
#include <vector>

// ROS stuff
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

// Boost stuff
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

// vrc stuff
#include <hubo/hubo_kinematics.h>
#include <utils/math_utils.h>
#include <utils/data_paths.h>
#include <utils/robot_configs.h>
#include <hubo/hubo_state.h>
#include <hubo/hubo_jacobian.h>

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

// GUI stuff
#include <gui/teleop_gui.h>
#include <pthread.h>

// SPACENAV stuff
#include "spnav.h"

// Fastrak stuff
#include "hubo_fastrak.h"

// Control stuff
#include <control/control.h>
#include <control/control_factory.h>
#include <control/arms.h>

//############################################################
//# Globals
//############################################################

// HUBO
hubo::hubo_state_t hubo_target;
hubo::hubo_state_t hubo_current;

// ROS
ros::NodeHandle* ros_node;

// CONTROLLERS
control::control_data_t* control_data;
control::control_t* spnav_control;
control::control_t* fastrak_control;

// PERIPHERALS
teleop::spnav_t spnav;
fastrak_t fastrak;

// GUI
pthread_t gui_thread;
gui::teleop_gui_t gui_window;
gui::params_t gui_params;

//############################################################
//# HUBO driver functions
//############################################################

void init_world_pose(robot::robot_state_t& robot)
{
    kinematics::Skeleton* hubo_skel = robot.robot();
    hubo_skel->setPose(robot.dart_pose());

    kinematics::BodyNode* left_foot = hubo_skel->getNode("leftFoot");
    kinematics::Shape* left_shoe = left_foot->getCollisionShape();

    Eigen::Matrix4d Twlf = left_foot->getWorldTransform();
    
    double pelvis_height = fabs(Twlf(2,3));

    Eigen::Isometry3d Twb;
    robot.get_body(Twb);
    Twb(2,3) = pelvis_height;
    robot.set_body(Twb);
}

bool init_sensors(const sensor_msgs::Joy::ConstPtr& joy, control::control_data_t* data)
{
    if(!spnav.sensor_update(joy))
        return false;

    return true;
}

void init_hubo(robot::robot_state_t& state, control::control_data_t* data)
{
    std::cout << "Initializing Hubo." << std::endl;

    // Data structures
    kinematics::Skeleton* hubo_skel = state.robot();
    Eigen::VectorXd hubo_dofs = state.dart_pose();

    // 1. Zeroing HUBO
    hubo_dofs.setZero();
    state.set_dart_pose(hubo_dofs);

    // 3. Set world pose
    init_world_pose(state);

    // 2. Get HUBO out of the singularity.
    std::vector<int> manip_enum = { robot::MANIP_L_HAND, robot::MANIP_R_HAND };
    Eigen::VectorXd arm_dofs(7);
    arm_dofs << 0, 0, 0, -M_PI/2, 0, M_PI/2, 0;
    for(int i=0; i < manip_enum.size(); i++) {
        state.set_manip(arm_dofs, manip_enum[i]);
    }

    // 3. Set target com to current
    hubo_skel->setPose( state.dart_pose() );
    data->com = hubo_skel->getWorldCOM();

    // 4. Set last command to current
    data->last_command = state.ros_pose();

    // 5. Set all manipulator targets to the reset pose
    std::vector<int> manip_ids = { robot::MANIP_L_HAND, robot::MANIP_R_HAND, robot::MANIP_L_FOOT, robot::MANIP_R_FOOT };
    std::vector<const char*> manip_names = { ROBOT_LEFT_HAND, ROBOT_RIGHT_HAND, ROBOT_LEFT_FOOT, ROBOT_RIGHT_FOOT };
    for(int i=0; i < manip_ids.size(); i++) {
        data->manip_xform[manip_ids[i]] = hubo_skel->getNode(manip_names[i])->getWorldTransform();
    }

    // 6. Calibrate fastrak to agree with this pose.
    fastrak.calibrate(state, data);
}

void update_sensors(const sensor_msgs::Joy::ConstPtr& joy, control::control_data_t* data)
{
    spnav.sensor_update(joy);
    spnav.get_teleop_data(data);
    fastrak.update_sensors(data);
}

bool get_key(char& ch) {
    if(gui_window.key != -1) {
        ch = gui_window.key;
        gui_window.key = -1;
        return true;
    }
    return false;
}

// does a keyboard update from gui
void do_keyboard(robot::robot_state_t& state, control::control_data_t* data)
{
    char key;
    bool ok = get_key(key);

    if(!ok) {
        return;
    }
    
    // Keys mappings
    std::map<char, std::string> spnav_keys, fastrak_keys;

    spnav_keys['a'] = "GRASP";
    spnav_keys['s'] = "BODY_XZP_FIX_LEGS";
    spnav_keys['d'] = "JOINT";
    spnav_keys['f'] = "ARM_JIT";
    spnav_keys['g'] = "ARM_AIK";
    spnav_keys['q'] = "NULL";
    //... add more controllers

    fastrak_keys['j'] = "ARM_BOTH_JIT";
    fastrak_keys['y'] = "NULL";

    // Do switching
    if(spnav_keys.count(key)) {
        delete spnav_control;
        spnav_control = control::get_factory(spnav_keys[key])->create();
        std::cout << "Switched SPNAV to " << spnav_control->name() << std::endl;
        return;
    }
    if(fastrak_keys.count(key)) {
        delete fastrak_control;
        fastrak_control = control::get_factory(fastrak_keys[key])->create();
        std::cout << "Switched FASTRAK to " << fastrak_control->name() << std::endl;
        return;
    }
    
    // General stuff
    switch (key) {
    case ' ':
    {
        int& ms = data->manip_side;
        ms = (ms+1)%2;
        std::cout << "Switch sides to " << (ms ? "LEFT":"RIGHT") << std::endl;
        break;
    }
    case 'c':
    {
        std::cout << "Zeroing fastrak" << std::endl;
        fastrak.calibrate(state, data);
        break;
    }
    case 'r':
    {
        std::cout << "Reseting poses\n";
        init_hubo(state, data);
        break;
    }
    default:
        break;
    }
}

// runs both the spnav and fastrak controllers
void do_control(robot::robot_state_t& target, control::control_data_t* data)
{
    if (spnav_control && data->joystick_ok) {
        spnav_control->run(target, data);
    }

    //FIXME: hubo_target should have the latest configuration.
    // If it fails, it should be reset? 

    if (fastrak_control && data->sensor_ok) {
        fastrak_control->run(target, data);
    }

    // FIXME: IF it fails i should be reset?

    data->joystick_ok = false; // reset joystick
    data->sensor_ok = false; // reset fastrak

    // FIXME: reset arms to match target location?
}

//############################################################
//# Main loop
//############################################################
void run(robot::robot_state_t& robot, const sensor_msgs::Joy::ConstPtr& joy, control::control_data_t* data) {
    static bool sensors_ready = false;
    if(!sensors_ready) {
        sensors_ready = init_sensors(joy, data); //< do not call init_sensors & update_sensors together
        return;
    }

    static bool hubo_ready = false;
    if(!hubo_ready) {
        init_hubo(robot, data);
        init_hubo(hubo_current, data); //< HACK hubo_current isn't use anyways
        hubo_ready = true;
    }

    do_keyboard(robot, data);

    update_sensors(joy, data);
    
    do_control(robot, data);
}

//############################################################
//# ROS loop
//############################################################
void spnav_handler(const sensor_msgs::Joy::ConstPtr& joy)
{
    run(hubo_target, joy, control_data);
}

//############################################################
//# main
//############################################################

int main(int argc, char *argv[])
{
    //###########################################################
    //# DART initialization
    DartLoader dart_loader;
    simulation::World* mWorld = dart_loader.parseWorld(VRC_DATA_PATH ROBOT_URDF);
    dynamics::SkeletonDynamics *hubo_skel = mWorld->getSkeleton(ROBOT_NAME); // grab pointer to robot
    hubo_target.init(hubo_skel);                     // init states
    hubo_current.init(hubo_skel);                    // init states
    hubo::hubo_kinematics_t hubo_kin;
    hubo_kin.init(hubo_skel); // use robot to init kinematics
    hubo::hubo_jacobian_t hubo_jac;
    hubo_jac.init(hubo_skel);  // init jacobians

    //###########################################################
    //# Control initilization
    control_data = new control::control_data_t();
    control_data->robot = hubo_skel;
    control_data->current = &hubo_current;
    control_data->kin = &hubo_kin;
    control_data->jac = &hubo_jac;
    control_data->manip_index = robot::MANIP_L_HAND;

    spnav_control = control::get_factory("GRASP")->create();
    fastrak_control = control::get_factory("ARM_BOTH_JIT")->create();

    //###########################################################
    //# GUI initialization
    control_data->manip_target = Eigen::Matrix4d::Identity();
    gui_params.goal = &control_data->manip_target;
    gui_params.current = &hubo_current;
    gui_params.target = &hubo_target;
    gui_params.draw_limits = true;
    gui_window.gui_params = &gui_params;
    pthread_create(&gui_thread, NULL, gui::teleop_gui_t::start_routine, &gui_window);    
    
    //###########################################################
    //# ROS initialization
    // initialize our ros node and block until we connect to the ROS master
    ros::init(argc, argv, "spacenav_teloperator");
    ros_node = new ros::NodeHandle();
    ros::Time last_ros_time_;
    bool wait = true;
    while (wait) {
        last_ros_time_ = ros::Time::now();
        if (last_ros_time_.toSec() > 0) wait = false;
    }

    //############################################################
    //# Joystick initialization
    ros::SubscribeOptions spnav_options = ros::SubscribeOptions::create<sensor_msgs::Joy>(
        "/spacenav/joy", 1, spnav_handler,
        ros::VoidPtr(), ros_node->getCallbackQueue());

    spnav_options.transport_hints = ros::TransportHints().unreliable();
    ros::Subscriber spnav_subscriber = ros_node->subscribe(spnav_options);

    //############################################################
    //# Loop
    ros::spin();

    return 0;
}