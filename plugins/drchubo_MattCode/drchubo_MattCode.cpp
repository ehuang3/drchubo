/**
 * @file drchubo_MattCode
 * @author A. Huaman, based on original ZMP code of M. Zucker
 * @date 2013/05/26
 */
#include "zmp/hubo-zmp.h"
#include "src/HuboPlus.h"
#include <math.h>
#include "mzcommon/MzGlutApp.h"
#include "mzcommon/TimeUtil.h"
#include <getopt.h>

#include "zmp/zmpwalkgenerator.h"
#include "zmp/footprint.h"

// Print stuff
#include <stdio.h>

#include "drchubo_MattCode.h"

using namespace fakerave;

namespace gazebo {

  const int drchubo_MattCode::mNumBodyDofs = 24; // Left Arm, Right Arm, Left Leg, Right Leg
  const int drchubo_MattCode::mNumJoints = 28;
  std::string drchubo_MattCode::mJointNames[] = {"drchubo::LSP", "drchubo::LSR", "drchubo::LSY", "drchubo::LEP", "drchubo::LWY", "drchubo::LWP", "drchubo::LWR",
			       "drchubo::RSP", "drchubo::RSR", "drchubo::RSY", "drchubo::REP", "drchubo::RWY", "drchubo::RWP", "drchubo::RWR",
			       "drchubo::LHY", "drchubo::LHR", "drchubo::LHP", "drchubo::LKP", "drchubo::LAP", "drchubo::LAR",
			       "drchubo::RHY", "drchubo::RHR", "drchubo::RHP", "drchubo::RKP", "drchubo::RAP", "drchubo::RAR",
			       "drchubo::NKY", "drchubo::NKP"};
  
  /**
   * @function drchubo_MattCode
   * @brief Constructor
   */
  drchubo_MattCode::drchubo_MattCode() {


  }

  /**
   * @function ~drchubo_MattCode
   * @brief Destructor
   */  
  drchubo_MattCode::~drchubo_MattCode() {

    event::Events::DisconnectWorldUpdateBegin( this->mUpdateConnection );     
  }
  
  /**
   * @function Load
   * @brief
   */
  void drchubo_MattCode::Load( physics::ModelPtr _model, 
				sdf::ElementPtr _sdf ) {

    printf( "*** Loading animation ***\n" );
    
    // Set gravity to false, otherwise the robot slowly falls
    _model->SetGravityMode( false );
    std::map<std::string, common::NumericAnimationPtr> joint_anim;
    common::NumericKeyFrame *joint_key;
    
    // Fill joint initialization
    double T = 25.0;
    for( int i = 0; i < mNumJoints; ++i ) {
      joint_anim[ mJointNames[i] ].reset( new common::NumericAnimation( "anim", T, true) );
    }
    
    // Generate ZMP trajectories
    generateZMPGait();

    //**********************************
    // Read file with trajectories
    //**********************************
    int numTrajPoints = mMzJointTraj.size();
    
    double t;
    double dt = T / (double)numTrajPoints;
    
    t = 0;
    for( int i = 0; i < numTrajPoints; ++i ) {
      
      std::vector<double> vals(mNumJoints);
      // Left Arm
      for( int j = 0; j <= 5; ++j ) {
	vals[j] = mMzJointTraj[i](j);
      }
      vals[6] = 0;
      // Right Arm
      for( int j = 7; j <= 12; ++j ) {
	vals[j] = mMzJointTraj[i](j-1);
      }
      vals[13] = 0;

      // Left and Right Legs
      for( int j = 14; j <= 25; ++j ) {
	vals[j] = mMzJointTraj[i](j-2);
      }
      vals[26] = 0;
      vals[27] = 0;

      for( int j = 0; j < mNumJoints; ++j ) {
	joint_key = joint_anim[ mJointNames[j] ]->CreateKeyFrame(t);
	joint_key->SetValue( vals[j] );
      }

      // Advance one time step
      t+=dt;
    }
    
    // Attach the animation to the model
    _model->SetJointAnimation( joint_anim );
    printf("End loading Joint animation \n");
    
    //*********************
    // POSE ANIMATION
    //*********************
    gazebo::common::PoseAnimationPtr pose_anim( new gazebo::common::PoseAnimation("test", T, true) );
      gazebo::common::PoseKeyFrame *pose_key;

      t = 0;
      float posx, posy, posz;

      for( int i = 0; i < numTrajPoints; ++i ) {

	pose_key = pose_anim->CreateKeyFrame(t);

	pose_key->SetTranslation(math::Vector3( posx, posy, posz + 1.25)); // 0.39
	pose_key->SetRotation(math::Quaternion(0, 0, 0.0));

	// Advance one time step
	t+=dt;
      }

      _model->SetAnimation( pose_anim );      
      printf( "** End loading Pose animation **\n" ); 
  }



/**
 * @function generateZMPGait
 */
void drchubo_MattCode::generateZMPGait() {
  printf(" Generate ZMP Gait \n");
  bool show_gui = false;
  bool use_ach = false;

  walktype walk_type = walk_canned;
  double walk_circle_radius = 5.0;
  double walk_dist = 20;

  double footsep_y = 0.085; // half of horizontal separation distance between feet
  double foot_liftoff_z = 0.05; // foot liftoff height

  double step_length = 0.3;
  bool walk_sideways = false;

  double com_height = 0.52; // height of COM above ANKLE
  double com_ik_ascl = 0;

  double zmpoff_y = 0; // lateral displacement between zmp and ankle
  double zmpoff_x = 0;

  double lookahead_time = 2.5;

  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.05;
  double single_support_time = 0.70;

  size_t max_step_count = 30;

  double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller

  // ZMPWalkGenerator::ik_error_sensitivity ik_sense = ZMPWalkGenerator::ik_strict;
  ZMPWalkGenerator::ik_error_sensitivity ik_sense = ZMPWalkGenerator::ik_sloppy;


  const char* hubofile = "/home/ana/Research/drchubo/plugins/drchubo_MattCode/myhubo.kinbody.xml";
  
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
  initContext.state.body_pos = vec3(0, 0, 0.85);
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

  for( int i = 0; i < walker.traj.size(); ++i ) {
    // Get stance foot
    Eigen::Matrix4d endT;

    if( walker.ref[i].stance == SINGLE_LEFT || walker.ref[i].stance == DOUBLE_LEFT ) {
      endT = tf2Mx( walker.ref[i].feet[0] );
    } else {
      endT = tf2Mx( walker.ref[i].feet[1] );
    }
    

  }
    
  std::cout << "Done and ready to step back and forth!" << std::endl;
}
 
  /**
   * @function tf2Mx
   */
  Eigen::Matrix4d drchubo_MattCode::tf2Mx( Transform3 _tf ) {

    Eigen::Matrix4d T4;
    mat4 m = _tf.matrix();
    for( int i = 0; i < 4; ++i ) {
      for( int j = 0; j < 4; ++j ) {
	T4(i, j) = m(i,j);
      }
    }
    
    return T4;
  }

/**
 * @function getdouble
 * @brief Helper to get a double value from a string
 */
  double drchubo_MattCode::getdouble(const char* str) {
    char* endptr;
    double d = strtod(str, &endptr);
    if (!endptr || *endptr) {
      std::cerr << "Error parsing number on command line!\n\n";
      exit(1);
    }
    return d;
  }
  
/**
 * @function getlong
 * @brief Helper to get a long value from a string
 */
long drchubo_MattCode::getlong(const char* str) {
  char* endptr;
  long d = strtol(str, &endptr, 10);
  if (!endptr || *endptr) {
    std::cerr << "Error parsing number on command line!\n\n";
    exit(1);
  }
  return d;
}

/**
 * @function getwalktype
 * @brief Helper to get a walk type from a string
 */
walktype drchubo_MattCode::getwalktype(const std::string& s) {
  if (s == "canned") {
    return walk_canned;
  } else if (s == "line") {
    return walk_line;
  } else if (s == "circle") {
    return walk_circle;
  } else {
    std::cerr << "bad walk type " << s << "\n";
    exit(1);
  }
}

/**
 * @function getiksense
 * @brief Helper to get ik from a string
 */
ZMPWalkGenerator::ik_error_sensitivity drchubo_MattCode::getiksense(const std::string& s) {
  if (s == "strict") {
    return ZMPWalkGenerator::ik_strict;
  } else if (s == "sloppy") {
    return ZMPWalkGenerator::ik_sloppy;
  } else if (s == "permissive") {
    return ZMPWalkGenerator::ik_swing_permissive;
  } else {
    std::cerr << "bad ik error sensitivity " << s << "\n";
    exit(1);
  }
}


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(drchubo_MattCode)
}
