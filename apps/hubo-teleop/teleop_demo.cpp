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

// Fastrak stuff
#include "hubo_fastrak.h"

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

dynamics::SkeletonDynamics* huboSkel;
hubo::hubo_kinematics_t huboKin;
hubo::hubo_state_t huboStateTarget;
hubo::hubo_state_t huboStateCurrent;
hubo::hubo_jacobian_t huboJac;

ros::Publisher topic_pub_joint_commands;

pthread_t gui_thread;
gui::teleop_gui_t gui_window;
gui::params_t gui_params;

bool gazebo_sim; //< we are simulation in gazebo

teleop::spnav_t spnav;

control::control_data_t* c_data;
control::control_t* controller;

fastrak_t fastrak;

//###########################################################
//###########################################################
//#### World orientation math
//###########################################################
//###########################################################
// Calculates the orientation of hubo's body given a foot position
void move_origin_to_feet(hubo::hubo_state_t& huboState)
{
    // 1. Get skeleton for computation
    kinematics::Skeleton *huboSkel = huboState.robot();
    huboSkel->setPose( huboState.dart_pose() );
    
    // 2. Get left foot, right foot, and body xforms from skeleton
    Eigen::Isometry3d Twlf, Twrf, Twb; //< xform world to: left foot, right foot, body
    Eigen::Isometry3d Two; //< world to new origin
    Twlf = huboSkel->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
    Twrf = huboSkel->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();
    Twb = huboSkel->getNode(ROBOT_BODY)->getWorldTransform();

    // 3. Orient hubo's body around his left foot.
    Eigen::Isometry3d Tlfb = Twlf.inverse() * Twb;
    huboState.set_body(Tlfb); // world origin at left foot
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
    
    // fastrak.update_sensors(c_data);

    //############################################################
    //### Init states
    static bool targetPoseInited = false;
    if (!targetPoseInited || c_data->triggers[1]) {
        // 1. Set target pose to match current
        std::cout << "Setting target pose." << std::endl;
        Eigen::VectorXd temp;
        huboStateCurrent.get_ros_pose(temp);
        huboStateTarget.set_ros_pose(temp);

        // 2. Set world origins to feet
        //move_origin_to_feet( huboStateTarget );
        //move_origin_to_feet( huboStateCurrent );

        // 3. Set target com to current
        huboSkel->setPose( huboStateCurrent.dart_pose() );
        c_data->com = huboSkel->getWorldCOM();

        // 4. Set last command to current
        c_data->last_command = huboStateCurrent.ros_pose();

        // 5. Set manip xforms
        c_data->manip_xform[robot::MANIP_L_HAND] = huboSkel->getNode(ROBOT_LEFT_HAND)->getWorldTransform();
        c_data->manip_xform[robot::MANIP_R_HAND] = huboSkel->getNode(ROBOT_RIGHT_HAND)->getWorldTransform();
        c_data->manip_xform[robot::MANIP_L_FOOT] = huboSkel->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
        c_data->manip_xform[robot::MANIP_R_FOOT] = huboSkel->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();

        // 6. Calibrate
        fastrak.calibrate(huboStateTarget, c_data);
        fastrak.update_sensors(c_data);
        std::cout << "Left calibrated position = \n" << c_data->sensor_tf[1].matrix() << std::endl;

        //
        targetPoseInited = true;
        
        return;
    }
    if (c_data->buttons[0]) {
        fastrak.update_sensors(c_data);
        std::cout << "Left calibrated position = \n" << c_data->sensor_tf[1].matrix() << std::endl;
    }

    if (!targetPoseInited)
        return;
    
    //############################################################
    //### Switch states
    if (gui_window.key != -1) {
        int key = gui_window.key;
        gui_window.key = -1;

        if('A' <= key && key <= 'Z') {
            // Do the state switch here...
            const std::vector<control::control_factory_t*>& F = control::factories();
            key = (key)%F.size();

            // Replace controller
            delete controller;
            controller = F[key]->create();
        
            // Let them know
            std::cout << "Switched to " << controller->name() << std::endl;
        }
        else {
            c_data->command_char = key;
            controller->change_mode(key, huboStateTarget);
        }
    }
    if (c_data->triggers[0]) {
        int& ms = c_data->manip_side;
        ms = (ms+1)%2;
        std::cout << "Switch sides to " << (ms ? "LEFT" : "RIGHT") << std::endl;
    }

    if (!c_data->buttons[0]) 
        return;
    
    //############################################################
    //### Run controller
    if (controller && (c_data->sensor_ok || c_data->joystick_ok))
        controller->run(huboStateTarget, c_data);

    //############################################################
    //### Send joint commands
    Eigen::VectorXd rospose;
    huboStateTarget.get_ros_pose(rospose);
    c_data->last_command = rospose;

    //############################################################
    //### Set all manipulator transforms to current location
    //FIXME: Does this cause any problems? (Likely will for AJIK)
    huboSkel->setPose(huboStateTarget.dart_pose());
    c_data->manip_xform[robot::MANIP_L_HAND] = huboSkel->getNode(ROBOT_LEFT_HAND)->getWorldTransform();
    c_data->manip_xform[robot::MANIP_R_HAND] = huboSkel->getNode(ROBOT_RIGHT_HAND)->getWorldTransform();
    // c_data->manip_xform[robot::MANIP_L_FOOT] = huboSkel->getNode(ROBOT_LEFT_FOOT)->getWorldTransform();
    // c_data->manip_xform[robot::MANIP_R_FOOT] = huboSkel->getNode(ROBOT_RIGHT_FOOT)->getWorldTransform();
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
    huboStateCurrent.set_ros_pose(r_pose);
    move_origin_to_feet(huboStateCurrent);
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
    mWorld = dart_loader.parseWorld(VRC_DATA_PATH ROBOT_URDF);
    huboSkel = mWorld->getSkeleton(ROBOT_NAME); // grab pointer to robot
    huboKin.init(huboSkel);                 // use robot to init kinematics
    huboStateTarget.init(huboSkel);         // init states
    huboStateCurrent.init(huboSkel);        // init states
    huboJac.init(huboSkel);                 // init jacobians
    huboKin.init(huboSkel);                 // init kinematics

    // Do not start at singularity
    Eigen::VectorXd leftQ(7);
    leftQ << 0, 0, 0, 0, 0, 0, 0;
    huboStateCurrent.set_manip(leftQ, robot::MANIP_L_HAND);

    //###########################################################
    //#### Controll initilization
    c_data = new control::control_data_t();
    c_data->robot = huboSkel;
    c_data->current = &huboStateCurrent;
    c_data->kin = &huboKin;
    c_data->jac = &huboJac;
    c_data->manip_index = robot::MANIP_L_HAND;

    controller = control::get_factory("ARM_JIT")->create();

    //###########################################################
    //#### GUI initialization
    c_data->manip_target = Eigen::Matrix4d::Identity();
    gui_params.goal = &c_data->manip_target;
    gui_params.current = &huboStateCurrent;
    gui_params.target = &huboStateTarget;
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

    //############################################################
    //### Fastrak initialization

    //###########################################################
    //#### Connect to simulation and start spinning
    ros::spin();

    return 0;
}
