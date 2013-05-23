/**
 * @file zmpWalkGenerator.cpp
 * @brief
 */
#include "zmpWalkGenerator.h"
#include "ZmpPreview.h"
#include "gait-timer.h"
#include "swing.h"

/**
 * @function ZMPWalkGenerator
 * @brief Constructor
 */
ZMPWalkGenerator::ZMPWalkGenerator( atlas::atlas_kinematics_t& _atlasKin,
				    kinematics::Skeleton* _atlasSkel,
				    zmp::ik_error_sensitivity _ik_sense,
				    double _com_height,
				    double _zmp_R,
				    double _zmpoff_x,
				    double _zmpoff_y,
				    double _com_ik_angle_weight,
				    double _min_single_support_time,
				    double _min_double_support_time,
				    double _walk_startup_time,
				    double _walk_shutdown_time,
				    double _step_height,
				    double _lookahead_time ) :
  mAtlasKin( _atlasKin ),
  mAtlasSkel( _atlasSkel ),
  ik_sense( _ik_sense ),
  com_height( _com_height ),
  zmp_R( _zmp_R ),
  zmpoff_x( _zmpoff_x ),
  zmpoff_y( _zmpoff_y ),
  com_ik_angle_weight( _com_ik_angle_weight ),
  min_single_support_time( _min_single_support_time ),
  min_double_support_time( _min_double_support_time ),
  walk_startup_time( _walk_startup_time ),
  walk_shutdown_time( _walk_shutdown_time ),
  step_height( _step_height ),
  lookahead_time( _lookahead_time ),
  haveInitContext( false )
{
}
  
    
/**
 * @function initialize
 * @brief These all modify the current context but do not immediately affect traj
 */
void ZMPWalkGenerator::initialize( const ZMPReferenceContext &_current ) {

  ref.clear();
  traj.clear();
  first_step_index = -1;
  initContext = _current;
  haveInitContext = true; // TODO: FIXME: CHECK THIS IN addFootstep and stayDogStay
}

/**
 * @function getLastRef
 * @brief Gets the last ZMPReferenceContext from ref or the initContext
 */
const ZMPReferenceContext& ZMPWalkGenerator::getLastRef() {
  return ref.empty() ? initContext : ref.back();
}

/**
 * @function stayDogStay
 * @brief these will add walk contexts to the back of ref and the new
 *         contexts don't have comX, comY, eX, eY however, the kstate will
 *         have body orientation set correctly and upper body joints
 */
void ZMPWalkGenerator::stayDogStay( size_t stay_ticks ) {

  ZMPReferenceContext cur = getLastRef();

  Eigen::Vector3d zmp_start( cur.pX, cur.pY, 0 );

  Eigen::Matrix3d rottemp = cur.feet[0].block(0,0,3,3);
  Eigen::Quaterniond rot_f0( rottemp );
  rottemp = cur.feet[1].block(0,0,3,3);
  Eigen::Quaterniond rot_f1( rottemp );

  Eigen::Affine3d midfoot( Eigen::Affine3d::Identity() );
  midfoot.rotate( rot_f0.slerp(0.05, rot_f1) );
  Eigen::Vector3d a = ( cur.feet[0].block(0,3,3,1) + cur.feet[1].block(0,3,3,1) );
  midfoot.translate( 0.5*a );

  
  Eigen::Vector3d zmp_end = midfoot * Eigen::Vector3d( zmpoff_x, zmpoff_y, 0 );

  stance_t double_stance = DOUBLE_LEFT;

  // TODO: get from timer!!
  size_t shift_ticks = TRAJ_FREQ_HZ * min_double_support_time;
  if (shift_ticks > stay_ticks) { shift_ticks = stay_ticks; }
  
  for ( size_t i = 0; i < shift_ticks; ++i ) {

    double u = double(i) / double(shift_ticks - 1);
    double c = sigmoid(u);
    
    cur.stance = double_stance;
    
    Eigen::Vector3d cur_zmp = zmp_start + (zmp_end - zmp_start) * c;

    cur.pX = cur_zmp.x();
    cur.pY = cur_zmp.y();

    ref.push_back(cur);

  }

  for ( size_t i = shift_ticks; i < stay_ticks; ++i ) {
    ref.push_back(cur);
  }

}

/**
 * @function addFootstep
 * @brief What do you think? Add a footstep :D
 */
void ZMPWalkGenerator::addFootstep( const Footprint& fp ) {

  // initialize our timer
  GaitTimer timer;
  timer.single_support_time = min_single_support_time;
  timer.double_support_time = min_double_support_time;
  timer.startup_time = walk_startup_time;
  timer.startup_time = walk_shutdown_time;

  // grab the initial position of the zmp
  const ZMPReferenceContext start_context = getLastRef();
  
  // figure out the stances for this movement
  stance_t double_stance = fp.is_left ? DOUBLE_RIGHT : DOUBLE_LEFT;
  stance_t single_stance = fp.is_left ? SINGLE_RIGHT : SINGLE_LEFT;
  if (step_height == 0) { single_stance = double_stance; }

  // figure out swing foot and stance foot for accessing
  int swing_foot = fp.is_left ? 0 : 1;
  int stance_foot = 1 - swing_foot;

  // figure out where our body is going to end up if we put our foot there
  Eigen::Matrix3d rottemp = start_context.feet[swing_foot].block(0,0,3,3);
  Eigen::Quaterniond start_q( rottemp );
  Eigen::Quaterniond fp_q( fp.getRotMat() );
  Eigen::Quaterniond body_rot_end = start_q.slerp( 0.5, fp_q );

  // figure out the start and end positions of the zmp
  Eigen::Vector3d zmp_end = (start_context.feet[stance_foot] * Eigen::Vector4d( zmpoff_x, zmpoff_y, 0,1 )).block(0,0,3,1);
  Eigen::Vector3d zmp_start = Eigen::Vector3d( start_context.pX, start_context.pY, 0 );
  
  size_t double_ticks = TRAJ_FREQ_HZ * min_double_support_time;
  size_t single_ticks = TRAJ_FREQ_HZ * min_single_support_time;

  for (size_t i = 0; i < double_ticks; i++) {
    // sigmoidally interpolate things like desired ZMP and body
    // rotation. we run the sigmoid across both double and single
    // support so we don't try to whip the body across for the
    // split-second we're in double support.
    double u = double(i) / double(double_ticks - 1);
    double c = sigmoid(u);
    
    double uu = double(i) / double(double_ticks + single_ticks - 1);
    double cc = sigmoid(uu);
    
    ZMPReferenceContext cur_context = start_context;
    cur_context.stance = double_stance;
    
    
    Eigen::Vector3d cur_zmp = zmp_start + (zmp_end - zmp_start) * c;
    cur_context.pX = cur_zmp.x();
    cur_context.pY = cur_zmp.y();

    cur_context.state.body_rot = (start_context.state.body_rot).slerp( cc, body_rot_end );
    
    ref.push_back(cur_context);
  }
  
  double swing_foot_traj[single_ticks][3];
  double swing_foot_angle[single_ticks];
  
  // note: i'm not using the swing_foot_angle stuff cause it seemed to not work
  swing2Cycloids(start_context.feet[swing_foot](0,3),
		 start_context.feet[swing_foot](1,3),
		 fp.theta(), // hack by now until I figure how to get the angle
		 fp.x(),
		 fp.y(),
		 fp.theta(),
		 single_ticks,
		 fp.is_left,
		 step_height,
		 swing_foot_traj,
		 swing_foot_angle);
  
  rottemp = start_context.feet[swing_foot].block(0,0,3,3);
  Eigen::Quaterniond foot_start_rot( rottemp );
  Eigen::Quaterniond foot_end_rot( fp.getRotMat() );
  
  // we want to have a small deadband at the front and end when we rotate the foot around
  // so it has no rotational velocity during takeoff and landing
  size_t rot_dead_ticks = 0.1 * TRAJ_FREQ_HZ; 
  
  if (single_ticks < 4*rot_dead_ticks) {
    rot_dead_ticks = single_ticks / 4;
  }
  
  assert(single_ticks > rot_dead_ticks * 2);
  
  size_t central_ticks = single_ticks - 2*rot_dead_ticks;
  
  for (size_t i = 0; i < single_ticks; i++) {
    
    double uu = double(i + double_ticks) / double(double_ticks + single_ticks - 1);
    double cc = sigmoid(uu);
    
    double ru;
    
    if (i < rot_dead_ticks) {
      ru = 0;
    } else if (i < rot_dead_ticks + central_ticks) {
      ru = double(i - rot_dead_ticks) / double(central_ticks - 1);
    } else {
      ru = 1;
    }
    

    ref.push_back(getLastRef());
    ZMPReferenceContext& cur_context = ref.back();
    cur_context.stance = single_stance;

    cur_context.state.body_rot = (start_context.state.body_rot).slerp( cc, body_rot_end );
    
    Eigen::Quaterniond fs_rot( foot_start_rot );
    Eigen::Quaterniond fe_rot( foot_end_rot );
    // According to a post - linear access the rotation matrix! for an Affine3d
    //    cur_context.feet[swing_foot].block(0,0,3,3) = (fs_rot.slerp( ru, fe_rot )).toRotationMatrix();
    cur_context.feet[swing_foot].block(0,0,3,3) = rottemp;
    cur_context.feet[swing_foot].block(0,3,3,1) = Eigen::Vector3d(swing_foot_traj[i][0],
								  swing_foot_traj[i][1],
								  swing_foot_traj[i][2] + fp.transform(2,3) );
  }
  
  // finally, update the first step variable if necessary
  if (first_step_index == size_t(-1)) { // we haven't taken a first step yet
    first_step_index = ref.size() - 1;
  }
  
}

/**
 * @function bakeIt
 */
void ZMPWalkGenerator::bakeIt() {

    traj.clear();
    runZMPPreview();
    runCOMIK();
    dumpTraj();

    // TODO: for thinking ahead
    //initContext = ref[first_step_index];

    ref.clear();
    haveInitContext = false;

}

/**
 * @functino applyComIK
 */
void ZMPWalkGenerator::applyComIK( ZMPReferenceContext& _cur ) {
  
  Eigen::Matrix4d Twb;
  Eigen::Matrix4d Twm[robot::robot_kinematics_t::NUM_MANIPULATORS];
  Eigen::VectorXd dofs;

  // Initialize
  dofs = mAtlasSkel->getPose().setZero();
  dofs = _cur.state.jvalues;

  // Set up body transform
  Twb = ( _cur.state.xform() ).matrix();

  // Set up desired CoM
  Eigen::Vector3d desiredCom = Eigen::Vector3d( _cur.comX[0], 
						_cur.comY[0], 
						_cur.comZ[0] );
  std::cout << "DESIRED COM: "<< desiredCom.transpose() << std::endl;
  // set mode for legs. Both arms are always fixed and invariant during walk
  _cur.ikMode[robot::robot_kinematics_t::MANIP_L_FOOT] = (_cur.stance == SINGLE_RIGHT) ? robot::robot_kinematics_t::IK_MODE_WORLD : robot::robot_kinematics_t::IK_MODE_SUPPORT;
  _cur.ikMode[robot::robot_kinematics_t::MANIP_R_FOOT] = (_cur.stance == SINGLE_LEFT) ? robot::robot_kinematics_t::IK_MODE_WORLD : robot::robot_kinematics_t::IK_MODE_SUPPORT;
  _cur.ikMode[robot::robot_kinematics_t::MANIP_L_HAND] = robot::robot_kinematics_t::IK_MODE_FIXED;
  _cur.ikMode[robot::robot_kinematics_t::MANIP_R_HAND] = robot::robot_kinematics_t::IK_MODE_FIXED;
   

  // and run IK. Everything we need goes straight into cur.state! cool.

  // Set manipulators transformations

  // Manipulator transforms
  Twm[robot::robot_kinematics_t::MANIP_L_FOOT] = _cur.feet[0].matrix();
  Twm[robot::robot_kinematics_t::MANIP_R_FOOT] = _cur.feet[1].matrix();

  
  // Sweet. Now do it!
  bool ok = mAtlasKin.comIK( mAtlasSkel,
			     desiredCom,
			     Twb,
			     _cur.ikMode,
			     Twm,
			     dofs );
    if (!ok) {
      std::cerr << "[applyComIK] IK FAILURE!\n\n";
      exit(1);
    }

    // Store the angles in jvalues
    else {
      _cur.state.jvalues = dofs;
    }

}

/**
 * @function dart2GazeboAngles
 */
void ZMPWalkGenerator::dart2GazeboAngles( const Eigen::VectorXd &_dartAngles,
					  double _trajAngles[] ) {
  
  int index = 0;
  
  // back_lbz, back_mby, back_ubx, neck_ay
  for( int i = 0; i < 4; ++i ) {
   _trajAngles[ index ] = 0; // Erased in new version. Let's set it to zero by now _dartAngles[ mAtlasKin.dof_misc[i] ];
    index++;
  }

  // left leg, right leg, left arm, right arm
  for( int i = 0; i < robot::robot_kinematics_t::NUM_MANIPULATORS; ++i ) {
    for( int j = 0; j < 6; ++j ) {
      _trajAngles[index] = _dartAngles[ mAtlasKin.dart_dof_ind[i][j] ];
      index++;
    }
  }

}

/**
 * @function refToTraj
 */
void ZMPWalkGenerator::refToTraj( const ZMPReferenceContext& ref,
				  zmp_traj_element_t& traj ) {
  
  // copy stance into output
  traj.stance = ref.stance;

  // copy joint angles from reference to output
  dart2GazeboAngles( ref.state.jvalues, traj.angles );

  // compute expected forces and torques and copy into output
  // NO BY NOW - SHOULD BE DONE BY DART, SOMEBODY FILL THIS


  // transform zmp and com into stance ankle reference frame, copy into output
  Eigen::Matrix4d stance_foot_trans = ref.stance == SINGLE_LEFT || ref.stance == DOUBLE_LEFT
    ? ref.feet[0] : ref.feet[1];

  Eigen::Vector3d zmp_in_world(ref.pX, ref.pY, 0);
  Eigen::Vector3d zmp_in_stance = ( stance_foot_trans.block(0,0,3,3).transpose() )*( zmp_in_world - stance_foot_trans.block(0,3,3,1) );

  traj.zmp[0] = zmp_in_stance.x();
  traj.zmp[1] = zmp_in_stance.y();

  for (size_t deriv = 0; deriv < 3; deriv++) {

    Eigen::Vector3d com_in_world( ref.comX[deriv], ref.comY[deriv], deriv == 0 ? com_height : 0);
    
    Eigen::Vector3d com_in_stance;
    if( deriv == 0 ) {
      com_in_stance = ( stance_foot_trans.block(0,0,3,3).transpose() )*
	( com_in_world - stance_foot_trans.block(0,3,3,1) );
    }
    else {
      com_in_stance = (stance_foot_trans.block(0,0,3,3)).transpose() * com_in_world;
    }  
    traj.com[0][deriv] = com_in_stance.x();
    traj.com[1][deriv] = com_in_stance.y();
    traj.com[2][deriv] = com_in_stance.z();
    
  }
  
}

/**
 * @function sigmoid
 */
double ZMPWalkGenerator::sigmoid( double x ) {
  return 3*x*x - 2*x*x*x;
}

/** 
 * @function runZMPPreview()
 * @brief run ZMP preview controller on entire reference and creates trajectory for COM pos/vel/acc in X and Y
 * @precondition: we have zmp reference values for x and y, initContext com and integrator error
 *               for initialization.
 * @postcondition: now we have set comX, comY, eX, eY for everything in ref.
 * @return: void
 */
void ZMPWalkGenerator::runZMPPreview(  ) {

    Eigen::Vector3d comX = initContext.comX; // initialize comX states to initial ZMPReferenceContext
    Eigen::Vector3d comY = initContext.comY; // initialize comY states to initial ZMPReferenceContext
    double eX = initContext.eX; //
    double eY = initContext.eY; //
    Eigen::ArrayXd zmprefX(ref.size());
    Eigen::ArrayXd zmprefY(ref.size());
    
    // initialize the zmp preview controller
    ZmpPreview preview(1.0/TRAJ_FREQ_HZ, com_height, lookahead_time*TRAJ_FREQ_HZ, zmp_R);

    // put all the zmp refs into eigen arrays in order to pass into the preview controller
    for(size_t i=0; i<ref.size(); i++) {
        zmprefX(i) = ref[i].pX;
        zmprefY(i) = ref[i].pY;
    }


    // generate COM position for each tick using zmp preview update
    for(size_t i = 0; i < ref.size(); i++) {
        ZMPReferenceContext& cur = ref[i];
        // run zmp preview controller to update COM states and integrator error
        preview.update(comX, eX, zmprefX.block(i, 0, ref.size()-i, 1));
	preview.update(comY, eY, zmprefY.block(i, 0, ref.size()-i, 1));
        cur.comX = comX; // set the ref comX pos/vel/acc for this tick
        cur.comY = comY; // set the ref comY pos/vel/acc for this tick
        cur.eX = eX;                 // set the X error for this tick
        cur.eY = eY;                 // set the Y error for this tick
    }
    
}

/**
 * @function: runCOMIK()
 * @brief: this runs the COM IK on every dang thing in reference to fill in the kstate
 * @precondition: reference is fully filled in
 * @postcondition: kstate is fully filled in
 * @return: void
 */
void ZMPWalkGenerator::runCOMIK(  ) {

    for( std::vector<ZMPReferenceContext>::iterator cur = ref.begin(); 
	 cur != ref.end(); cur++ ) {
      applyComIK(*cur);
    }
}


/**
 * @function dumpTraj
 * @brief: picks everything important out of ref which is now fully specified and creates a trajectory
 * @precondition: we have ref fully filled in
 * @postcondition: forces and torque are calculated and everything is transformed into stance ankle reference frame
 * @return: void
 */
void ZMPWalkGenerator::dumpTraj(  ) {

  for( std::vector<ZMPReferenceContext>::iterator cur_ref = ref.begin(); 
       cur_ref != ref.end(); 
       cur_ref++ ) {
    
    // make output
    traj.push_back( zmp_traj_element_t() );
    refToTraj( *cur_ref, traj.back() );
  }

}

/**
 * @function printDebugInfo
 */
void ZMPWalkGenerator::printDebugInfo() {
  
  FILE* zmp; FILE* leftFoot; FILE* rightFoot; FILE* com; 

  zmp = fopen( "zmp.txt", "w" );
  leftFoot = fopen( "leftFoot.txt", "w" );
  rightFoot = fopen( "rightFoot.txt", "w" );
  //com = fopen( "com.txt", "w" );

  for( int i = 0; i < ref.size(); ++i ) {

    fprintf( zmp, "%d  %f %f \n", i, ref[i].pX, ref[i].pY );

    Eigen::Vector3d lf = ref[i].feet[0].matrix().block(0,3,3,1);
    Eigen::Vector3d rf = ref[i].feet[1].matrix().block(0,3,3,1);
    fprintf( leftFoot, "%d %f %f %f %d \n", i, lf(0), lf(1), lf(2), ref[i].stance );
    fprintf( rightFoot, "%d %f %f %f \n", i, rf(0), rf(1), rf(2) );    
  }
  
  fclose( zmp );
  fclose( leftFoot );
  fclose( rightFoot );
  //fclose( com );
}


/* Local Variables: */
/* mode: c++ */
/* c-basic-offset: 2 */
/* End: */
