/**
 * @file zmpnode
 * @author A. Huaman, based on original ZMP code of M. Zucker
 * @date 2013/05/29
 */
#include "zmp/hubo-zmp.h"
#include "src/HuboPlus.h"
#include <math.h>
#include "mzcommon/MzGlutApp.h"
#include "mzcommon/TimeUtil.h"
#include <getopt.h>

#include "zmp/zmpwalkgenerator.h"
#include "zmp/footprint.h"

// ROS Msg 
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <DRC_msgs/PoseStampedArray.h>

// Print stuff
#include <stdio.h>

// Dart stuff
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <dynamics/SkeletonDynamics.h>

// Header
#include "zmpnode.h"

// Paths
#include "src/utils/data_paths.h"

using namespace fakerave;


const int zmpnode::mNumBodyDofs = 24; // Left Arm, Right Arm, Left Leg, Right Leg
const int zmpnode::mNumJoints = 35;
std::string zmpnode::mJointNames[] = {"drchubo::LSP", "drchubo::LSR", "drchubo::LSY", "drchubo::LEP", "drchubo::LWY", "drchubo::LWP", "drchubo::LWR",
				      "drchubo::RSP", "drchubo::RSR", "drchubo::RSY", "drchubo::REP", "drchubo::RWY", "drchubo::RWP", "drchubo::RWR",
				      "drchubo::LHY", "drchubo::LHR", "drchubo::LHP", "drchubo::LKP", "drchubo::LAP", "drchubo::LAR",
				      "drchubo::RHY", "drchubo::RHR", "drchubo::RHP", "drchubo::RKP", "drchubo::RAP", "drchubo::RAR",
				      "drchubo::TSY", "drchubo::NKY", "drchubo::NKP",
				      "drchubo::LF1", "drchubo::LF2", "drchubo::LF3",
				      "drchubo::RF1", "drchubo::RF2", "drchubo::RF3"};
/**
 * @function zmpnode
 * @brief Constructor
 */
zmpnode::zmpnode() {
  mdt = 0.005;
  ankleOffset = 0.20; // 0.18 - IF YOU CHANGE THIS, ALSO CHANGE THE ankleOffset of DRCPlugin
}

/**
 * @function ~zmpnode
 * @brief Destructor
 */  
zmpnode::~zmpnode() {
}

/**
 * @function getJointTrajMsg
 * @brief Get a trajectory_msgs JointTrajectory message
 */
trajectory_msgs::JointTrajectory zmpnode::getJointTrajMsg() {  
  printf( "*** Getting Trajectory msg ***\n" );
  
  //**********************************
  // JOINT ANIMATION
  //**********************************
  // Append names
  trajectory_msgs::JointTrajectory jt;
  jt.header.stamp = ros::Time::now();
  jt.header.frame_id = "drchubo::Body_Torso";
  
  for( int i = 0; i < mNumJoints; ++i ) {
    jt.joint_names.push_back( mJointNames[i] );  
  }
  
  int numTrajPoints = mMzJointTraj.size();
  double t = 0;
  
  for( int i = 0; i < numTrajPoints; ++i ) {
    
    trajectory_msgs::JointTrajectoryPoint p;
    
    std::vector<double> vals(mNumJoints, 0.0);
    // Left Arm
    for( int j = 0; j <= 5; ++j ) {
      vals[j] = mMzJointTraj[i](j);
    }
    // Right Arm
    for( int j = 7; j <= 12; ++j ) {
      vals[j] = mMzJointTraj[i](j-1);
    }
    // Left and Right Legs
    for( int j = 14; j <= mNumJoints; ++j ) {
      vals[j] = mMzJointTraj[i](j-2);
    }
    // Set all joints
    for( int j = 0; j < mNumJoints; ++j ) {
      p.positions.push_back( vals[j] );
    }
    // Append to trajectory
    jt.points.push_back(p);
    
    // Set duration
    jt.points[i].time_from_start = ros::Duration().fromSec(t);
    
    // Advance one time step
    t+=mdt;
  }
  
  return jt;
  
}

/**
 * @function getPoseTrajMsg
 */  
DRC_msgs::PoseStampedArray zmpnode::getPoseTrajMsg() {

  //*********************
  // POSE ANIMATION
  //*********************
  DRC_msgs::PoseStampedArray pt;

  int numTrajPoints = mMzJointTraj.size();
  double t;
  
  t = 0;
  for( int i = 0; i < numTrajPoints; ++i ) {
    
    geometry_msgs::PoseStamped p;
    p.pose.position.x = mRootX[i];
    p.pose.position.y = mRootY[i];
    p.pose.position.z = mRootZ[i];
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    
        // Set the time
    p.header.stamp = ros::Time(t);
    
    // Append to pose trajectory
    pt.poses.push_back( p ); 
    
    // Advance one time step
    t+=mdt;
  }
  
  return pt;
  
}

/**
 * @function getPoseJointTrajMsg
 */
DRC_msgs::PoseJointTrajectory zmpnode::getPoseJointTrajMsg(  geometry_msgs::PosePtr _initPose,
							     sensor_msgs::JointStatePtr _initJointState,
                                                             double _smoothTransitionTime ) {

  //**********************************
  // JOINT AND POSE ANIMATION
  //**********************************
  // Append names
  DRC_msgs::PoseJointTrajectory pjt;
  
  for( int i = 0; i < mNumJoints; ++i ) {
    pjt.joint_names.push_back( mJointNames[i] );  
  }

  double t = 0;
  // ----------------------------------------------------------------
  // Set the first smooth transition
  printf("Setting the first smooth transition \n");
  trajectory_msgs::JointTrajectoryPoint jt0;
  geometry_msgs::Pose p0;
  double smoothT = _smoothTransitionTime; // Time for it to get to initial position

  // Get angle values
  std::vector<double> vals0( mNumJoints, 0 );
  for( int i = 0; i < mNumJoints; ++i ) {

    for( int j = 0; j < _initJointState->name.size(); ++j ) {
      if( strcmp( mJointNames[i].c_str(), _initJointState->name[j].c_str()) == 0 ) {
	vals0[i] = _initJointState->position[j];
	//	printf("Setting %s joint with value in joint state of index %d - Size state: %d  \n", mJointNames[i].c_str(), j, );
	break;
      }
    }

  }
  printf("Setting first animation step \n");
  // Set first animation step
  for( int j = 0; j < mNumJoints; ++j ) {
    jt0.positions.push_back( vals0[j] );
  }

  p0.position.x = _initPose->position.x;
  p0.position.y = _initPose->position.y;
  p0.position.z = _initPose->position.z;  // + 1.18 - DEFAULT FROM FLOOR
  p0.orientation.x = _initPose->rotation.x;
  p0.orientation.y = _initPose->rotation.y;
  p0.orientation.z = _initPose->rotation.z;
  p0.orientation.w = _initPose->rotation.w;
  
  // Append to trajectory
  pjt.points.push_back(jt0);
  pjt.poses.push_back(p0);
  
  // Set duration
  pjt.points[0].time_from_start = ros::Duration().fromSec(0);
  
  // Add transition time
  t += smoothT;

  // --------------------------------------------------
  // Normal animation
  printf("Normal animation start procedure\n");

  int numTrajPoints = mMzJointTraj.size();

  
  for( int i = 0; i < numTrajPoints; ++i ) {
    
    trajectory_msgs::JointTrajectoryPoint jt;
    geometry_msgs::Pose p;
    
    std::vector<double> vals(mNumJoints, 0.0);
    // Left Arm
    for( int j = 0; j <= 5; ++j ) {
      vals[j] = mMzJointTraj[i](j);
    }
    // Right Arm
    for( int j = 7; j <= 12; ++j ) {
      vals[j] = mMzJointTraj[i](j-1);
    }
    // Left and Right Legs (do not change the 25 - MATT CODE RETURNS 26 values)
    for( int j = 14; j <= 25; ++j ) {
      vals[j] = mMzJointTraj[i](j-2);
    }

    //** Set all joints
    for( int j = 0; j < mNumJoints; ++j ) {
      jt.positions.push_back( vals[j] );
    }

    //** Set poses
    p.position.x = mRootX[i] + _initPose->position.x;
    p.position.y = mRootY[i] + _initPose->position.y;
    p.position.z = mRootZ[i]; // + 1.18 - DEFAULT FROM FLOOR
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = 0;
    p.orientation.w = 1;
    
    // Append to trajectory
    pjt.points.push_back(jt);
    pjt.poses.push_back(p);
    
    // Set duration (+1 for the initial smooth time)
    pjt.points[i+1].time_from_start = ros::Duration().fromSec(t);
    
    // Advance one time step
    t+=mdt;
  }

  // Set this info at the end, just to be in order
  pjt.header.stamp = ros::Time::now();
  pjt.header.frame_id = "drchubo::Body_Torso";

  printf("End of animation processing, returning \n");

  return pjt;
}


/**
 * @function generateZMPGait
 * @brief Executes Matt's code
 */
  void zmpnode::generateZMPGait( size_t _max_step_count,
                                 double _step_length ) {
  printf(" Generate ZMP Gait \n");
  bool show_gui = false;
  bool use_ach = false;

  walktype walk_type = walk_canned;
  double walk_circle_radius = 5.0;
  double walk_dist = 10;

  double footsep_y = 0.10;//0.0985; // half of horizontal separation distance between feet
  double foot_liftoff_z = 0.05; // foot liftoff height

  bool walk_sideways = false;

  double com_height = 0.48; // 0.48// height of COM above ANKLE
  double com_ik_ascl = 0;

  double zmpoff_y = 0; // lateral displacement between zmp and ankle
  double zmpoff_x = 0;

  double lookahead_time = 2.5;

  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.05;
  double single_support_time = 0.70;

  // Values set by arguments
  double max_step_count = _max_step_count;
  double step_length = _step_length;


  double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller

  // ZMPWalkGenerator::ik_error_sensitivity ik_sense = ZMPWalkGenerator::ik_strict;
  ZMPWalkGenerator::ik_error_sensitivity ik_sense = ZMPWalkGenerator::ik_sloppy;


  const char* hubofile( DRC_ROOT_PATH "apps/zmpnode/matt_stuff/myhubo.kinbody.xml" );
  
  HuboPlus hplus(hubofile);
  printf( "Loaded correctly, I hope \n"); 

  //////////////////////////////////////////////////////////////////////
  // build initial state

  // the actual state
  ZMPWalkGenerator walker(hplus,
			  ik_sense,
                          com_height,
                          zmp_jerk_penalty,
			  zmpoff_x,
			  zmpoff_y,
                          com_ik_ascl,
                          single_support_time,
                          double_support_time,
                          startup_time,
                          shutdown_time,
                          foot_liftoff_z,
			  lookahead_time
			  );
  ZMPReferenceContext initContext;
  
  // helper variables and classes
  const KinBody& kbody = hplus.kbody;
  const JointLookup& jl = hplus.jl;
  double deg = M_PI/180; // for converting from degrees to radians

  // fill in the kstate
  initContext.state.body_pos = vec3(0, 0, 0.7); //0.85
  initContext.state.body_rot = quat();
  initContext.state.jvalues.resize(kbody.joints.size(), 0.0);
  initContext.state.jvalues[jl("LSR")] =  15*deg;
  initContext.state.jvalues[jl("RSR")] = -15*deg;
  initContext.state.jvalues[jl("LSP")] =  20*deg;
  initContext.state.jvalues[jl("RSP")] =  20*deg;
  initContext.state.jvalues[jl("LEP")] = -40*deg;
  initContext.state.jvalues[jl("REP")] = -40*deg;
  
  // build and fill in the initial foot positions

  Transform3 starting_location(quat::fromAxisAngle(vec3(0,0,1), 0));
  initContext.feet[0] = Transform3(starting_location.rotation(), starting_location * vec3(0, footsep_y, 0));
  initContext.feet[1] = Transform3(starting_location.rotation(), starting_location * vec3(0, -footsep_y, 0));

  // fill in the rest
  initContext.stance = DOUBLE_LEFT;
  initContext.comX = Eigen::Vector3d(zmpoff_x, 0.0, 0.0);
  initContext.comY = Eigen::Vector3d(0.0, 0.0, 0.0);
  initContext.eX = 0.0;
  initContext.eY = 0.0;
  initContext.pX = 0.0;
  initContext.pY = 0.0;

  // apply COM IK for init context
  walker.applyComIK(initContext);

  // Add initContext as, well, first context
  walker.initialize(initContext);

  //////////////////////////////////////////////////////////////////////
  // build ourselves some footprints
  
  Footprint initLeftFoot = Footprint(initContext.feet[0], true);
  Footprint initRightFoot = Footprint(initContext.feet[1], false);

  std::vector<Footprint> footprints;

  switch (walk_type) {
  case walk_circle: {

    double circle_max_step_angle = M_PI / 12.0; // maximum angle between steps TODO: FIXME: add to cmd line??
  
    footprints = walkCircle(walk_circle_radius,
			    walk_dist,
			    footsep_y,
			    step_length,
			    circle_max_step_angle,
			    &initLeftFoot,
			    &initRightFoot,
			    false);

    break;
  }

  case walk_line: {

    footprints = walkLine(walk_dist, footsep_y,
			  step_length,
			  &initLeftFoot,
			  &initRightFoot,
			  false);
    break;

  }

  default: {

    double cur_x[2] = { 0, 0 };
    double cur_y[2] = { 0, 0 };

    cur_y[0] =  footsep_y;
    cur_y[1] = -footsep_y;
    
    for (size_t i=0; i<max_step_count; ++i) {
      bool is_left = i%2;
      if (walk_sideways && step_length < 0) { is_left = !is_left; }
      int swing = is_left ? 0 : 1;
      int stance = 1-swing;
      if (walk_sideways) {
	cur_y[swing] -= step_length;
      } else {
	if (i + 1 == max_step_count) {
	  cur_x[swing] = cur_x[stance];
	} else {
	  cur_x[swing] = cur_x[stance] + 0.5*step_length;
	}
      }
      footprints.push_back(Footprint(cur_x[swing], cur_y[swing], 0, is_left));
    }

    break;
  }
  }

  if (footprints.size() > max_step_count) {
    footprints.resize(max_step_count);
  }

  //////////////////////////////////////////////////////////////////////
  // and then build up the walker


  walker.stayDogStay(startup_time * TRAJ_FREQ_HZ);


  for(std::vector<Footprint>::iterator it = footprints.begin(); it != footprints.end(); it++) {
    walker.addFootstep(*it);
  }

  walker.stayDogStay(shutdown_time * TRAJ_FREQ_HZ);
  

  //////////////////////////////////////////////////////////////////////
  // have the walker run preview control and pass on the output
  
  walker.bakeIt();
  
  
  // Save joints
  std::vector<int> jointOrderFake( mNumBodyDofs );
  std::cout << "The trajectory size was: "<< walker.traj.size() << std::endl;
  // Got this from HuboPlus.cpp line 318 (hnames)
  // Still not sure why this is the way it is...but that is it so don't ask me
  jointOrderFake[0] = 4; jointOrderFake[1] = 5;
  jointOrderFake[2] = 6; jointOrderFake[3] = 7;
  jointOrderFake[4] = 8; jointOrderFake[5] = 10;
  
  jointOrderFake[6] = 11; jointOrderFake[7] = 12;
  jointOrderFake[8] = 13; jointOrderFake[9] = 14;
  jointOrderFake[10] = 15; jointOrderFake[11] = 17;
  
  jointOrderFake[12] = 19; jointOrderFake[13] = 20;
  jointOrderFake[14] = 21; jointOrderFake[15] = 22;
  jointOrderFake[16] = 23; jointOrderFake[17] = 24;
  
  jointOrderFake[18] = 26; jointOrderFake[19] = 27;
  jointOrderFake[20] = 28; jointOrderFake[21] = 29;
  jointOrderFake[22] = 30; jointOrderFake[23] = 31;
  
  // Store path
  mMzJointTraj.resize( walker.traj.size() );
  for( int i = 0; i < walker.traj.size(); ++i ) {
    Eigen::VectorXd waypoint = Eigen::VectorXd::Zero( mNumBodyDofs );
    
    for( int j = 0; j < mNumBodyDofs; ++j ) {
      waypoint(j) = walker.traj[i].angles[ jointOrderFake[j] ];
    }		
    
    mMzJointTraj[i] = waypoint;
  }

  // Store root position

  // Create a DART Skeleton
  kinematics::Skeleton *captain;

  DartLoader dart_loader;
  simulation::World *mWorld = dart_loader.parseWorld(DRC_DATA_PATH "models/drchubo-master/hubo_world.urdf");
  
  captain = mWorld->getSkeleton("drchubo");
  if( captain == NULL ) {
    captain = mWorld->getSkeleton("golem_hubo");
    printf("Loaded skeleton under the name golem_hubo \n" );
  } else {
    printf("Loaded skeleton under the name drchubo \n" );
  }

  // Get the Dof indices for left arm, right arm, left leg, right leg
  std::string mBodyDofNames[] = {"Body_LSP", "Body_LSR", "Body_LSY", "Body_LEP", "Body_LWY", "Body_LWP",
				 "Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP",
				 "Body_LHY", "Body_LHR", "Body_LHP", "Body_LKP", "Body_LAP", "Body_LAR",
				 "Body_RHY", "Body_RHR", "Body_RHP", "Body_RKP", "Body_RAP", "Body_RAR"};
  
  // Store the indices for the Body Dofs
  // To set the skeleton to that config
  std::vector<int> mBodyDofs( mNumBodyDofs );
  for(int i = 0; i < mBodyDofs.size(); i++) {
    mBodyDofs[i] = captain->getNode( mBodyDofNames[i].c_str())->getDof(0)->getSkelIndex();
  }

  mRootX.resize(0);   mRootY.resize(0);   mRootZ.resize(0);
  int stance;

  Eigen::Matrix4d footT;
  Eigen::Matrix4d jointTinv;
  Eigen::Matrix4d rootT;
  double Zrelative;

  // i = 0 - First value
  stance = walker.ref[0].stance;

  if( stance == SINGLE_LEFT || stance == DOUBLE_LEFT ) {
    footT = tf2Mx( walker.ref[0].feet[0] );   
    captain->setConfig( mBodyDofs, mMzJointTraj[0] );
    jointTinv = captain->getNode("Body_LAR")->getWorldInvTransform();
    Zrelative = captain->getNode("Body_Torso")->getWorldTransform()(2,3) - captain->getNode("Body_LAR")->getWorldTransform()(2,3);
  } else {
    footT = tf2Mx( walker.ref[0].feet[1] );   
    captain->setConfig( mBodyDofs, mMzJointTraj[0] );
    jointTinv = captain->getNode("Body_RAR")->getWorldInvTransform();
    Zrelative = captain->getNode("Body_Torso")->getWorldTransform()(2,3) - captain->getNode("Body_RAR")->getWorldTransform()(2,3);
  }

    rootT = footT*jointTinv;
    //Get initial offset x and y and apply it
    double offx = rootT(0,3);
    double offy = rootT(1,3);  

    mRootX.push_back( rootT(0,3) - offx );
    mRootY.push_back( rootT(1,3) - offy );
    mRootZ.push_back( Zrelative + ankleOffset );
 
    printf("0 Location: %f %f %f \n", rootT(0,3), rootT(1,3), Zrelative + ankleOffset );

    // Rest of values
  for( int i = 1; i < walker.traj.size(); ++i ) {

    stance = walker.ref[i].stance;
  
    if( stance == SINGLE_LEFT || stance == DOUBLE_LEFT ) {
      if( stance == DOUBLE_LEFT ) {
	footT = rootT*captain->getNode("Body_LAR")->getWorldTransform();
	Zrelative = captain->getNode("Body_Torso")->getWorldTransform()(2,3) - captain->getNode("Body_LAR")->getWorldTransform()(2,3);
      }
      captain->setConfig( mBodyDofs, mMzJointTraj[i] );
      jointTinv = captain->getNode("Body_LAR")->getWorldInvTransform();
      Zrelative = captain->getNode("Body_Torso")->getWorldTransform()(2,3) - captain->getNode("Body_LAR")->getWorldTransform()(2,3);
    } else {
      if( stance == DOUBLE_RIGHT ) {
	footT = rootT*captain->getNode("Body_RAR")->getWorldTransform();
    Zrelative = captain->getNode("Body_Torso")->getWorldTransform()(2,3) - captain->getNode("Body_RAR")->getWorldTransform()(2,3);
      }
      captain->setConfig( mBodyDofs, mMzJointTraj[i] );
      jointTinv = captain->getNode("Body_RAR")->getWorldInvTransform();
    Zrelative = captain->getNode("Body_Torso")->getWorldTransform()(2,3) - captain->getNode("Body_RAR")->getWorldTransform()(2,3);
      
    }
    
    rootT = footT*jointTinv;
    mRootX.push_back( rootT(0,3) - offx );
    mRootY.push_back( rootT(1,3) - offy );
    mRootZ.push_back( Zrelative + ankleOffset );
  }
    
  std::cout << "Done and ready to step back and forth!" << std::endl;
}
 
/**
 * @function tf2Mx
 */
Eigen::Matrix4d zmpnode::tf2Mx( Transform3 _tf ) {
  
  Eigen::Matrix4d T4;
  mat4 m = _tf.matrix();
  for( int i = 0; i < 4; ++i ) {
    for( int j = 0; j < 4; ++j ) {
      T4(i, j) = m(i,j);
    }
  }
  
  return T4;
}

