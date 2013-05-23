/**
 * @file comikTest.cpp
 */
#include <adaptedZMP/zmpWalkGenerator.h>
#include <adaptedZMP/atlas_zmp.h>
#include <adaptedZMP/footprint.h>

#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/BodyNode.h>
#include <robotics/parser/dart_parser/DartLoader.h>

#include <utils/data_paths.h>

bool initKinematics( atlas::atlas_kinematics_t &_atlasKin,
		     dynamics::SkeletonDynamics* &_atlasSkel );
ZMPReferenceContext initWalkingContext( atlas::atlas_kinematics_t &_atlasKin,
					dynamics::SkeletonDynamics* _atlasSkel,
					const Eigen::Matrix4d &_Twb );
void initWalkGenerator( atlas::atlas_kinematics_t &_atlasKin,
			dynamics::SkeletonDynamics* &_atlasSkel,
			ZMPWalkGenerator* &_walker,
			const Eigen::Matrix4d &_Twb );

Eigen::Matrix4d getLimbTransW( atlas::atlas_kinematics_t _ak, kinematics::Skeleton *_atlas,
			const Eigen::Matrix4d &_Twb, 
			int _limb ); 

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  atlas::atlas_kinematics_t gAtlasKin;
  dynamics::SkeletonDynamics* gAtlasSkel;
  ZMPWalkGenerator* gWalker;

  // Init kinematics
  initKinematics( gAtlasKin,
		  gAtlasSkel );

  // Init walking
  Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();

  initWalkGenerator( gAtlasKin,
		     gAtlasSkel,
		     gWalker,
		     Twb );

  // Init walking context
  ZMPReferenceContext initContext;
  initContext = initWalkingContext( gAtlasKin,
				    gAtlasSkel,
				    Twb );

  // initialize
  gWalker->initialize(initContext);
    
  //-- build footprints
  std::vector<Footprint> footprints;
  
  // Walk in a straight line
  double walk_dist = 5;
  double footsep_y = ( getLimbTransW( gAtlasKin, gAtlasSkel, Twb, robot::robot_kinematics_t::MANIP_L_FOOT)(1,3) - 
		       getLimbTransW( gAtlasKin, gAtlasSkel, Twb, robot::robot_kinematics_t::MANIP_R_FOOT)(1,3) ) / 2.0;
  double step_length = 0.3;

  Footprint initLeftFoot = Footprint( initContext.feet[0], true );

  footprints = walkLine( walk_dist, footsep_y,
			 step_length,
			 initLeftFoot );
  
  //-- and then build up the walker

  // Foot steps
  for(std::vector<Footprint>::iterator it = footprints.begin(); it != footprints.end(); it++) {
    gWalker->addFootstep(*it);
  }

  // Print
  gWalker->printDebugInfo();

  return 0;
}

/**
 * @function getLimbTransW
 */
Eigen::Matrix4d getLimbTransW( atlas::atlas_kinematics_t _ak, kinematics::Skeleton *_atlas,
			const Eigen::Matrix4d &_Twb, 
			int _limb ) {
	Eigen::Matrix4d transW;

	Eigen::Matrix4d transB;
	Eigen::Vector6d angles;

	// Set the angles
	int nDofsNum = _atlas->getNumDofs();
	Eigen::VectorXd dofs(nDofsNum);
	dofs = _atlas->getPose();

	for (int i = 0; i < 6; i++)
		angles(i) = dofs(_ak.dart_dof_ind[_limb][i]);

  // Get the FK      
  switch( _limb ) {
		case robot::robot_kinematics_t::MANIP_L_FOOT:
			transB = _ak.legFK( angles, true );
			break;

		case robot::robot_kinematics_t::MANIP_R_FOOT:
			transB = _ak.legFK( angles, false );
			break;	
	}


	transW = _Twb*transB;

	return transW;
} 


/**
 * @function initKinematics
 */
bool initKinematics( atlas::atlas_kinematics_t &_atlasKin,
		     dynamics::SkeletonDynamics* &_atlasSkel ) {

  // Load urdf
  DartLoader dl;
  simulation::World *world = dl.parseWorld( VRC_DATA_PATH "models/atlas/atlas_world.urdf" );

  // Save skeleton
  _atlasSkel = world->getSkeleton( "atlas" );

  if( !_atlasSkel ) {
    std::cout<<"[ERROR] Atlas Skeleton no loaded, exiting!"<< std::endl;
    return false;
  }
  
  // Initialize _atlasKin
  _atlasKin.init( _atlasSkel );

  std::cout << "\n Finished init Kinematics "<<std::endl;
  return true;
}

/**
 * @function initWalkGenerator
 */
void initWalkGenerator( atlas::atlas_kinematics_t &_atlasKin,
			dynamics::SkeletonDynamics* &_atlasSkel,
			ZMPWalkGenerator* &_walker,
			const Eigen::Matrix4d &_Twb ) {

  /**< half of horizontal separation distance between feet */

    
  /**<  height of COM above ANKLE */
  double com_height = _atlasSkel->getWorldCOM()(2) - 
    getLimbTransW( _atlasKin, _atlasSkel, _Twb, robot::robot_kinematics_t::MANIP_L_FOOT )(2,3);

  double foot_liftoff_z = 0.05; // foot liftoff height
  
  bool walk_sideways = false;  
  walktype walk_type = walk_canned;
  double walk_circle_radius = 5.0;
  
  double com_ik_ascl = 0;
  
  double zmpoff_y = 0;
  double zmpoff_x = 0;
  
  double lookahead_time = 2.5;
  
  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.05;
  double single_support_time = 0.70;
  
  size_t max_step_count = 25;
  
  double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller
  
  zmp::ik_error_sensitivity ik_sense = zmp::ik_strict;

  //-- build initial state

  // the actual state
  _walker = new ZMPWalkGenerator( _atlasKin,
				  _atlasSkel,
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
				  lookahead_time );

}


/**
 * @function initWalkingContext
 */
ZMPReferenceContext initWalkingContext( atlas::atlas_kinematics_t &_atlasKin,
					dynamics::SkeletonDynamics* _atlasSkel,
					const Eigen::Matrix4d &_Twb ) {

  ZMPReferenceContext initContext;

  // Set body transform
  initContext.state.setXform( _Twb );

  // Set stance
  initContext.stance = DOUBLE_LEFT;
  
  // Set IK Mode
  robot::robot_kinematics_t::IK_Mode mode[robot::robot_kinematics_t::NUM_MANIPULATORS];
  mode[robot::robot_kinematics_t::MANIP_L_FOOT] = robot::robot_kinematics_t::IK_MODE_SUPPORT;
  mode[robot::robot_kinematics_t::MANIP_R_FOOT] = robot::robot_kinematics_t::IK_MODE_WORLD;
  mode[robot::robot_kinematics_t::MANIP_L_HAND] = robot::robot_kinematics_t::IK_MODE_FIXED;
  mode[robot::robot_kinematics_t::MANIP_R_HAND] = robot::robot_kinematics_t::IK_MODE_FIXED;

  // Set CoM
  Eigen::Vector3d com = _atlasSkel->getWorldCOM();
  initContext.comX << com(0), 0, 0;
  initContext.comY << com(1), 0, 0;
  initContext.comZ << com(2), 0, 0;

  // Set desired ZMP
  initContext.eX = 0.0; initContext.eY = 0.0;
  initContext.pX = 0.0; initContext.pY = 0.0;  

  // fill in the kstate
  initContext.state.jvalues.resize( _atlasSkel->getNumDofs() );
  initContext.state.jvalues = _atlasSkel->getPose();
  
  // Store feet positions
  initContext.feet[0] = getLimbTransW( _atlasKin, _atlasSkel, _Twb, robot::robot_kinematics_t::MANIP_L_FOOT );
  initContext.feet[1] = getLimbTransW( _atlasKin, _atlasSkel, _Twb, robot::robot_kinematics_t::MANIP_R_FOOT );
  
  return initContext;
}
