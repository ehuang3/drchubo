/**
 * @file Ingress.h
 */

#include "Ingress.h"

#include <tf_conversions/tf_kdl.h>
#include <atlas_msgs/WalkDemoGoal.h>

// Useful constants
const float Ingress::sKp_pos[28] = { 20.0, 4000.0, 2000.0, 20.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 
				     5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 2000.0, 1000.0, 200.0, 200.0, 
				     50.0, 100.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0};

const double Ingress::sInit_pos[28] = { 2.438504816382192e-05, 0.0015186156379058957, 9.983908967114985e-06, -0.0010675729718059301, 
					-0.0003740221436601132, 0.06201673671603203, -0.2333149015903473, 0.5181407332420349, 
					-0.27610817551612854, -0.062101610004901886, 0.00035181696875952184, -0.06218484416604042, 
					-0.2332201600074768, 0.51811283826828, -0.2762000858783722, 0.06211360543966293, 
					0.29983898997306824, -1.303462266921997, 2.0007927417755127, 0.49823325872421265, 
					0.0003098883025813848, -0.0044272784143686295, 0.29982614517211914, 1.3034454584121704, 
					2.000779867172241, -0.498238742351532, 0.0003156556049361825, 0.004448802210390568 };


/**
 * @function init
 * @brief Initialize ROS stuff
 */
void Ingress::init() {

  ros::spinOnce();
  mNode = new ros::NodeHandle();

  mNumJoints = 28;
  
  // Set up publishers / subscribers
  mAc_pub = mNode->advertise<atlas_msgs::AtlasCommand>( "atlas/atlas_command", 10, false );
  mAsic_pub = mNode->advertise<atlas_msgs::AtlasSimInterfaceCommand>("atlas/atlas_sim_interface_command", 1, false );

  mAsis_sub = mNode->subscribe("atlas/atlas_sim_interface_state", 1000, &Ingress::state_cb, this );
  mAs_sub = mNode->subscribe("atlas/atlas_state", 1000, &Ingress::as_cb, this );

  // Wait for subscribers to hook up, lest they miss our commands
  ros::Duration(2.0).sleep();
  
  ros::spinOnce();
  
}


/**
 * @function getToPose
 */
void Ingress::getToPose( std::vector<double> _goalPose,
			 double _time,
			 int _frequency ) {

  // Pose variable
  std::vector<double> dPose( mNumJoints );

  // Message
  atlas_msgs::AtlasCommand user_msg;

  // Message generic 
  // Assign default Ks 
  user_msg.velocity = std::vector<double>( mNumJoints, 0.0 );
  user_msg.effort = std::vector<double>( mNumJoints, 0.0 );

  user_msg.kp_position = std::vector<float>( mNumJoints, 0.0 );

  for( int i = 0; i < mNumJoints; ++i ) {
    user_msg.kp_position[i] = sKp_pos[i];
  }

  user_msg.ki_position = std::vector<float>( mNumJoints, 0.0 );
  user_msg.kd_position = std::vector<float>( mNumJoints, 0.0 );

  // Bump up kp_velocity to reduce the jerkiness of the transition
  user_msg.kp_velocity = std::vector<float>( mNumJoints, 50.0 );
  user_msg.i_effort_min = std::vector<float>( mNumJoints, 0.0 );
  user_msg.i_effort_max = std::vector<float>( mNumJoints, 0.0 );

  // Set k_effort = [1] to indicate that we want all joints under user control
  user_msg.k_effort = std::vector<uint8_t>( mNumJoints, 255 );


  // Loop until you get to goalPose (I am not checking the final error)
  double dt = 1.0 / (double) _frequency;
  int steps = (int) ( _time / dt );
  
  // Spin to get current position in mAs_msg
  ros::spinOnce();
  std::vector<double> start(mNumJoints);
  for( int i = 0; i < mNumJoints; ++i ) {
    start[i] = mAs_msg.position[i];
  }

  // Send messages
  for( int i = 0; i < steps; ++i ) {

    ros::spinOnce();
    double factor = ((double)i / (double)steps);
    // Find the diff to cover in one timestep
    for( int j = 0; j < _goalPose.size(); ++j ) {
      dPose[j] = (_goalPose[j] - start[j])*factor + start[j];
    }

    user_msg.header.stamp = ros::Time::now();
    user_msg.position = dPose;
    
    // Publish it
    mAc_pub.publish( user_msg );

    // Give it time to be executed
    ros::Duration(dt).sleep();
  }

  printf( "Done Get To Pose \n" );
  ros::spinOnce();
  ros::Duration(1.0).sleep();

  for( int i = 0; i < mNumJoints; ++i ) {
    printf("Start: %f Final joint: %f Goal was: %f \n", start[i], mAs_msg.position[i], _goalPose[i] );
  }

}

/**
 * @function switchToBdiStandMode
 */
void Ingress::switchToBdiStandMode() {

  atlas_msgs::AtlasSimInterfaceCommand stand_msg;

  // Always insert current time
  stand_msg.header.stamp = ros::Time::now();

  // Tell it to stand
  stand_msg.behavior = stand_msg.STAND_PREP;
  
  // Set k_effort = [255] to indicate that we still want all joints under user
  // control.  The stand behavior needs a few iterations before it start
  // outputting force values.
  stand_msg.k_effort = std::vector<uint8_t>( mNumJoints, 255 );
  
  // Publish and give time to take effect
  printf( "[USER CONTROL] Warming up BDI stand...\n" );
  mAsic_pub.publish( stand_msg );
  ros::Duration(2.0).sleep();
    
  // Now switch to stand
  stand_msg.behavior = stand_msg.STAND;

  // Set k_effort = [0] to indicate that we want all joints under BDI control
  stand_msg.k_effort = std::vector<uint8_t>( mNumJoints, 0 );

  // Publish and give time to take effect
  printf( "[BDI CONTROL] Standing... \n" );
  mAsic_pub.publish( stand_msg );
  ros::Duration(4.0).sleep();
  printf("Done initialization of BDI Stand Mode \n");
}


/**
 * @function as_cb
 * @brief Atlas State callback. Store atlas status (joint state) info
 */
void Ingress::as_cb( const atlas_msgs::AtlasState& _as_msg ) {

  mAs_msg = _as_msg;
}

/**
 * @function state_cb
 * @brief Callback
 */
void Ingress::state_cb( const atlas_msgs::AtlasSimInterfaceState& _asis_msg ) {
  mAsis_msg = _asis_msg;
}


/**
 * @function ingress_1
 * @brief Hard-coded blind walk towards the car in VRC Task 1
 */
void Ingress::ingress_1() {

  std::vector<double> initPose( mNumJoints );

  // **************************************************
  // Step 1: Go to stand-prep pose under user control
  // **************************************************
  printf("Start PREP STAND \n");
  for( int i = 0; i < mNumJoints; ++i ) {
    initPose[i] = sInit_pos[i];
    }
  getToPose( initPose );
  
  // ***********************************
  // Step 2: Request BDI stand mode
  // ***********************************
  printf("Switch to BDI Stand Mode \n");
  switchToBdiStandMode();
  
  // *********************************************************************************
  // Step 3: Bend knees a bit
  // *********************************************************************************
  atlas_msgs::AtlasCommand slight_movement_msg;
  // Always insert current time
  slight_movement_msg.header.stamp = ros::Time::now();
  // Start with 0.0 and set values for the joints that we want to control
  slight_movement_msg.position = std::vector<double>( mNumJoints, 0.0 );

  slight_movement_msg.position[atlas_msgs::AtlasState::l_leg_kny] = 0.6;

  slight_movement_msg.position[atlas_msgs::AtlasState::r_leg_lhy] = -0.5;
  slight_movement_msg.position[atlas_msgs::AtlasState::r_leg_kny] = 0.6;
  slight_movement_msg.position[atlas_msgs::AtlasState::r_leg_lax] = 0.5;

  slight_movement_msg.kp_position = std::vector<float>( mNumJoints, 0.0 );
  for( int i = 0; i < mNumJoints; ++i ) {
    slight_movement_msg.kp_position[i] = sKp_pos[i];
  }

  // Make ankle stronger
  slight_movement_msg.kp_position[atlas_msgs::AtlasState::r_leg_lax] = 2*sKp_pos[atlas_msgs::AtlasState::r_leg_lax];

  slight_movement_msg.ki_position = std::vector<float>( mNumJoints, 0.0 );
  slight_movement_msg.kd_position = std::vector<float>( mNumJoints, 0.0 );
  // Bump up kp_velocity to reduce the jerkiness of the transition
  slight_movement_msg.kp_velocity = std::vector<float>( mNumJoints, 50.0 );
  slight_movement_msg.i_effort_min = std::vector<float>( mNumJoints, 0.0 );
  slight_movement_msg.i_effort_max = std::vector<float>( mNumJoints, 0.0 ); 

  // Set k_effort = [1] for the joints that we want to control.
  // BDI has control of the other joints
  slight_movement_msg.k_effort = std::vector<uint8_t>( mNumJoints, 0 );
  slight_movement_msg.k_effort[atlas_msgs::AtlasState::l_leg_kny] = 255;

  slight_movement_msg.k_effort[atlas_msgs::AtlasState::r_leg_lhy] = 255;
  slight_movement_msg.k_effort[atlas_msgs::AtlasState::r_leg_kny] = 255;
  slight_movement_msg.k_effort[atlas_msgs::AtlasState::r_leg_lax] = 255;

  // Publish and give time to take effect
  printf( "[USER/BDI] Command bending knees...\n" );
  mAc_pub.publish(slight_movement_msg);
  ros::Duration(3.0).sleep();
  

}



/**
 * @function demo
 * @brief Tutorial code
 */
void Ingress::demo() {

  init();

  ros::spinOnce();
  std::vector<double> start(mNumJoints);
  for( int i = 0; i < mNumJoints; ++i ) {
    start[i] = mAs_msg.position[i];
  }
  

  // **********************************************
  // Step 0: Go to home pose under user control
  // **********************************************
  atlas_msgs::AtlasCommand home_msg;
  // Always insert current time
  home_msg.header.stamp = ros::Time::now();
  // Assign some position and gain values that will get us there.
  home_msg.position = std::vector<double>( mNumJoints, 0.0 );
  home_msg.velocity = std::vector<double>( mNumJoints, 0.0 );
  home_msg.effort = std::vector<double>( mNumJoints, 0.0 );

  home_msg.kp_position = std::vector<float>( mNumJoints, 0.0 );
  float kp_pos[28] = {20.0, 4000.0, 2000.0, 20.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 
		      5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 2000.0, 1000.0, 200.0, 200.0, 
		      50.0, 100.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0};
  for( int i = 0; i < mNumJoints; ++i ) {
    home_msg.kp_position[i] = kp_pos[i];
  }

  home_msg.ki_position = std::vector<float>( mNumJoints, 0.0 );
  home_msg.kd_position = std::vector<float>( mNumJoints, 0.0 );

  // Bump up kp_velocity to reduce the jerkiness of the transition
  home_msg.kp_velocity = std::vector<float>( mNumJoints, 50.0 );
  home_msg.i_effort_min = std::vector<float>( mNumJoints, 0.0 );
  home_msg.i_effort_max = std::vector<float>( mNumJoints, 0.0 );
  // Set k_effort = [1] to indicate that we want all joints under user control
  home_msg.k_effort = std::vector<uint8_t>( mNumJoints, 255 );

  // Publish and give time to take effect
  printf("[USER] Going to home position... \n");
  mAc_pub.publish( home_msg );
  ros::Duration(2.0).sleep();
  printf("Start step 1 \n");

  printf( "Done Get To Pose DEMO, check errors \n" );
  ros::spinOnce();
  ros::Duration(1.0).sleep();

  for( int i = 0; i < mNumJoints; ++i ) {
    printf("Start: %f Final joint: %f Goal was: %f \n", start[i], mAs_msg.position[i],0.0 );
  }


  
  // **************************************************
  // Step 1: Go to stand-prep pose under user control
  // **************************************************
  atlas_msgs::AtlasCommand stand_prep_msg;
  // Always insert current time
  stand_prep_msg.header.stamp = ros::Time::now();
  // Assign some position and gain values that will get us there.
  stand_prep_msg.position = std::vector<double>( mNumJoints, 0.0 );
  double pos_init[28] = {2.438504816382192e-05, 0.0015186156379058957, 9.983908967114985e-06, -0.0010675729718059301, 
			 -0.0003740221436601132, 0.06201673671603203, -0.2333149015903473, 0.5181407332420349, 
			 -0.27610817551612854, -0.062101610004901886, 0.00035181696875952184, -0.06218484416604042, 
			 -0.2332201600074768, 0.51811283826828, -0.2762000858783722, 0.06211360543966293, 
			 0.29983898997306824, -1.303462266921997, 2.0007927417755127, 0.49823325872421265, 
			 0.0003098883025813848, -0.0044272784143686295, 0.29982614517211914, 1.3034454584121704, 
			 2.000779867172241, -0.498238742351532, 0.0003156556049361825, 0.004448802210390568 };
  for( int i = 0; i < mNumJoints; ++i ) {
    stand_prep_msg.position[i] = pos_init[i];
  }

  stand_prep_msg.velocity = std::vector<double>( mNumJoints, 0.0 );
  stand_prep_msg.effort = std::vector<double>( mNumJoints, 0.0 );

  stand_prep_msg.kp_position = std::vector<float>( mNumJoints, 0.0 );
  for( int i = 0; i < mNumJoints; ++i ) {
    stand_prep_msg.kp_position[i] = kp_pos[i];
  }

  stand_prep_msg.ki_position = std::vector<float>( mNumJoints, 0.0 );
  stand_prep_msg.kd_position = std::vector<float>( mNumJoints, 0.0 );
  // Bump up kp_velocity to reduce the jerkiness of the transition
  stand_prep_msg.kp_velocity = std::vector<float>( mNumJoints, 50.0 );
  stand_prep_msg.i_effort_min = std::vector<float>( mNumJoints, 0.0 );
  stand_prep_msg.i_effort_max = std::vector<float>( mNumJoints, 0.0 );
  // Set k_effort = [1] to indicate that we want all joints under user control
  stand_prep_msg.k_effort =  std::vector<uint8_t>( mNumJoints, 255 );
  // Publish and give time to take effect
  printf( "[USER] Going to stand prep position...\n" );
  mAc_pub.publish( stand_prep_msg );
  ros::Duration(2.0).sleep();


  printf("Start step 2 \n");
  
  // ***********************************
  // Step 2: Request BDI stand mode
  // ***********************************
  atlas_msgs::AtlasSimInterfaceCommand stand_msg;
  // Always insert current time
  stand_msg.header.stamp = ros::Time::now();

  // Tell it to stand
  stand_msg.behavior = stand_msg.STAND_PREP;

  // Set k_effort = [255] to indicate that we still want all joints under user
  // control.  The stand behavior needs a few iterations before it start
  // outputting force values.
  stand_msg.k_effort = std::vector<uint8_t>( mNumJoints, 255 );

  // Publish and give time to take effect
  printf( "[USER] Warming up BDI stand...\n" );
  mAsic_pub.publish( stand_msg );
  ros::Duration(2.0).sleep();
    
  // Now switch to stand
  stand_msg.behavior = stand_msg.STAND;
  // Set k_effort = [0] to indicate that we want all joints under BDI control
  stand_msg.k_effort = std::vector<uint8_t>( mNumJoints, 0 );
  // Publish and give time to take effect
  printf( "[BDI] Standing... \n" );
  mAsic_pub.publish( stand_msg );
  ros::Duration(4.0).sleep();

  printf( "Start step 3: Move head and arms slightly \n" );
  // *********************************************************************************
  // Step 3: Move the arms and head a little (not too much; don't want to fall over)
  // *********************************************************************************
  atlas_msgs::AtlasCommand slight_movement_msg;
  // Always insert current time
  slight_movement_msg.header.stamp = ros::Time::now();
  // Start with 0.0 and set values for the joints that we want to control
  slight_movement_msg.position = std::vector<double>( mNumJoints, 0.0 );
  slight_movement_msg.position[atlas_msgs::AtlasState::neck_ay] = 1.0;
  
  slight_movement_msg.position[atlas_msgs::AtlasState::l_arm_ely] = 1.0;
  slight_movement_msg.position[atlas_msgs::AtlasState::l_arm_mwx] = 1.0;
  slight_movement_msg.position[atlas_msgs::AtlasState::r_arm_ely] = -1.0;
  slight_movement_msg.position[atlas_msgs::AtlasState::r_arm_mwx] = -1.0;
  slight_movement_msg.velocity = std::vector<double>( mNumJoints, 0.0 );
  slight_movement_msg.effort = std::vector<double>( mNumJoints, 0.0 );

  slight_movement_msg.kp_position = std::vector<float>( mNumJoints, 0.0 );
  for( int i = 0; i < mNumJoints; ++i ) {
    slight_movement_msg.kp_position[i] = kp_pos[i];
  }


  slight_movement_msg.ki_position = std::vector<float>( mNumJoints, 0.0 );
  slight_movement_msg.kd_position = std::vector<float>( mNumJoints, 0.0 );
  // Bump up kp_velocity to reduce the jerkiness of the transition
  stand_prep_msg.kp_velocity = std::vector<float>( mNumJoints, 50.0 );
  slight_movement_msg.i_effort_min = std::vector<float>( mNumJoints, 0.0 );
  slight_movement_msg.i_effort_max = std::vector<float>( mNumJoints, 0.0 ); 
  // Set k_effort = [1] for the joints that we want to control.
  // BDI has control of the other joints
  slight_movement_msg.k_effort = std::vector<uint8_t>( mNumJoints, 0 );
  slight_movement_msg.k_effort[atlas_msgs::AtlasState::neck_ay] = 255;
  slight_movement_msg.k_effort[atlas_msgs::AtlasState::l_arm_ely] = 255;
  slight_movement_msg.k_effort[atlas_msgs::AtlasState::l_arm_mwx] = 255;
  slight_movement_msg.k_effort[atlas_msgs::AtlasState::r_arm_ely] = 255;
  slight_movement_msg.k_effort[atlas_msgs::AtlasState::r_arm_mwx] = 255;
  // Publish and give time to take effect
  printf( "[USER/BDI] Command neck and arms...\n" );
  mAc_pub.publish(slight_movement_msg);
  ros::Duration(3.0).sleep();
  
  printf("Start step 4: Light walking \n");
  // ***************************
  // Step 4: Request BDI walk
  // ***************************
  atlas_msgs::AtlasSimInterfaceCommand walk_msg;
  // Always insert current time
  walk_msg.header.stamp = ros::Time::now();
  // Tell it to walk
  walk_msg.behavior = walk_msg.WALK;
  walk_msg.walk_params.use_demo_walk = false;
  // Fill in some steps
  for( int i=0; i < 4; ++i ) {
    atlas_msgs::AtlasBehaviorStepData step_data;
    // Steps are indexed starting at 1
    step_data.step_index = i+1;
    // 0 = left, 1 = right
    step_data.foot_index = i%2;
    // 0.3 is a good number
    step_data.swing_height = 0.3;
    // 0.63 is a good number
    step_data.duration = 0.63;
    // We'll specify desired foot poses in ego-centric frame then
    // transform them into the robot's world frame.
    // Match feet so that we end with them together
    step_data.pose.position.x = (1 +i)*0.3;
    // Step 0.15m to either side of center, alternating with feet
    if (i%2==0) { step_data.pose.position.y = 0.14; } 
    else { step_data.pose.position.y = -0.14; }
    step_data.pose.position.z = 0.0;

    // Point those feet straight ahead
    step_data.pose.orientation.x = 0.0;
    step_data.pose.orientation.y = 0.0;
    step_data.pose.orientation.z = 0.0;
    step_data.pose.orientation.w = 1.0;
    // Transform this foot pose according to robot's
    // current estimated pose in the world, which is a combination of IMU
    // and internal position estimation.
    // http://www.ros.org/wiki/kdl/Tutorials/Frame%20transformations%20%28Python%29
    ros::spinOnce();
    KDL::Frame f1, f2, f;
    tf::PoseMsgToKDL( step_data.pose, f1 );
    f2 = KDL::Frame( KDL::Rotation::Quaternion( mAs_msg.orientation.x,
					        mAs_msg.orientation.y,
						mAs_msg.orientation.z,
						mAs_msg.orientation.w ),
		     KDL::Vector( mAsis_msg.pos_est.position.x,
				  mAsis_msg.pos_est.position.y,
				  mAsis_msg.pos_est.position.z) );
    f = f2 * f1;

    tf::PoseKDLToMsg( f, step_data.pose );
    printf("\n Step [%d]: %f, %f, %f Orientation: %f %f %f %f \n", i, step_data.pose.position.x,
	   step_data.pose.position.y,
	   step_data.pose.position.z,
	   step_data.pose.orientation.x,
	   step_data.pose.orientation.y,
	   step_data.pose.orientation.z,
	   step_data.pose.orientation.w );
    
    walk_msg.walk_params.step_queue[i] = step_data;
  }
  // Use the same k_effort from the last step, to retain user control over some
  // joints. BDI has control of the other joints.
  walk_msg.k_effort = slight_movement_msg.k_effort;
  // Publish and give time to take effect
  printf( "[USER/BDI] Walking... \n" );
  mAsic_pub.publish( walk_msg );
  ros::Duration(6.0).sleep();
  
  // Step 5: Go back to home pose under user control

  // Always insert current time
  home_msg.header.stamp = ros::Time::now();
  // Assign some position and gain values that will get us there.
  home_msg.position = std::vector<double>( mNumJoints, 0.0 );
  home_msg.velocity = std::vector<double>( mNumJoints, 0.0 );
  home_msg.effort = std::vector<double>( mNumJoints, 0.0 );
  home_msg.kp_position = std::vector<float>( mNumJoints, 0.0 );
  for( int i = 0; i < mNumJoints; ++i ) {
    home_msg.kp_position[i] = kp_pos[i];
  }

  home_msg.ki_position = std::vector<float>( mNumJoints, 0.0 );
  home_msg.kd_position = std::vector<float>( mNumJoints, 0.0 );
  // Bump up kp_velocity to reduce the jerkiness of the transition
  home_msg.kp_velocity = std::vector<float>( mNumJoints, 50.0 );
  home_msg.i_effort_min = std::vector<float>( mNumJoints, 0.0 );
  home_msg.i_effort_max = std::vector<float>( mNumJoints, 0.0 ); 
  // Set k_effort = [1] to indicate that we want all joints under user control
  home_msg.k_effort = std::vector<uint8_t>( mNumJoints, 255 );
  // Publish and give time to take effect
  printf( "[USER] Going to home position... \n" );
  mAc_pub.publish( home_msg );
  ros::Duration(2.0).sleep();
  
  
  printf("Demo is over \n");
}

