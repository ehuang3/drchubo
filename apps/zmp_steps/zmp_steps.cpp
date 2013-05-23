/**
 * @file matahari.cpp
 */
#include <adaptedZMP/zmpWalkGenerator.h>
#include <adaptedZMP/atlas_zmp.h>
#include <adaptedZMP/footprint.h>

#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/BodyNode.h>
#include <robotics/parser/dart_parser/DartLoader.h>

#include <utils/data_paths.h> // For ATLAS_DATA_PATH

bool mainDebug = true;

Eigen::Matrix4d getLimbTransW( atlas::atlas_kinematics_t _ak, kinematics::Skeleton *_atlas,
			const Eigen::Matrix4d &_Twb, 
			int _limb ); 

Eigen::Vector3d getCOMW(kinematics::Skeleton *_atlas, const Eigen::Matrix4d &_Twb);

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  //-- Initialize atlasKin
  atlas::atlas_kinematics_t atlasKin;
  dynamics::SkeletonDynamics *atlasSkel;
  
  DartLoader dl;
  simulation::World *world = dl.parseWorld( VRC_DATA_PATH "models/atlas/atlas_world.urdf" );
  atlasSkel = world->getSkeleton( "atlas" );

  if( !atlasSkel ) {
    std::cout<<" Atlas Skeleton no loaded, exiting!"<< std::endl;
    return 1;
  }
  
  atlasKin.init( atlasSkel );


  //-- Set some walking variables

  // helper variables and classes
  double deg = M_PI/180; // for converting from degrees to radians

  // Set initial pose to find footsep
  Eigen::VectorXd initPose = atlasSkel->getPose().setZero();
  initPose( 20 ) =  15*deg; initPose( 22 ) = -15*deg; // LSR / RSR
  initPose( 15 ) =  20*deg; initPose( 17 ) =  20*deg; // LSP / RSP
  initPose( 29 ) = -40*deg; initPose( 30 ) = -40*deg; // LEP / REP
  atlasSkel->setPose( initPose );

  /**< half of horizontal separation distance between feet */
  double footsep_y = ( getLimbTransW( atlasKin, atlasSkel, Eigen::Matrix4d::Identity(), robot::robot_kinematics_t::MANIP_L_FOOT)(1,3) - 
		       getLimbTransW( atlasKin, atlasSkel, Eigen::Matrix4d::Identity(), robot::robot_kinematics_t::MANIP_R_FOOT)(1,3) ) / 2.0;
    
  /**<  height of COM above ANKLE */
  double com_height = getCOMW( atlasSkel, Eigen::Matrix4d::Identity() )(2) - getLimbTransW( atlasKin, atlasSkel, 
													      Eigen::Matrix4d::Identity(), 
													      robot::robot_kinematics_t::MANIP_L_FOOT )(2,3);

  double foot_liftoff_z = 0.05; // foot liftoff height
  double step_length = 0.3;

  bool walk_sideways = false;  
  walktype walk_type = walk_canned;
  double walk_circle_radius = 5.0;
  double walk_dist = 5;

  double com_ik_ascl = 0;
  
  double zmpoff_y = 0; // lateral displacement between zmp and ankle
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
  ZMPWalkGenerator walker( atlasKin,
			   atlasSkel,
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


  // Fill init context
  ZMPReferenceContext initContext;


  // init body transformation
  Eigen::Matrix4d Twb; Eigen::Affine3d bodyXform;
  Twb = Eigen::Matrix4d::Identity();
  bodyXform.matrix() = Twb;

  initContext.state.setXform( bodyXform );
  initContext.stance = DOUBLE_LEFT;

  // init CoM
  Eigen::Vector3d initCOM = getCOMW( atlasSkel, Twb );
  initContext.comX << initCOM(0), 0, 0;
  initContext.comY << initCOM(1), 0, 0;
  initContext.comZ << initCOM(2), 0, 0;
  std::cout << " \n CoM with zero pose: "<< initContext.comX(0)<<","<<initContext.comY(0)<<","<<initContext.comZ(0) << std::endl;

  // init desired ZMP
  initContext.eX = 0.0; initContext.eY = 0.0;
  initContext.pX = 0.0; initContext.pY = 0.0;  

  // fill in the kstate
  initContext.state.jvalues.resize( atlasSkel->getNumDofs() );
  initContext.state.jvalues = initPose;
  
  // Store feet positions
  initContext.feet[0].matrix() = getLimbTransW( atlasKin, atlasSkel, Twb, robot::robot_kinematics_t::MANIP_L_FOOT );
  initContext.feet[1].matrix() = getLimbTransW( atlasKin, atlasSkel, Twb, robot::robot_kinematics_t::MANIP_R_FOOT );

  // We want the robot to have its CoM down
  initContext.comZ(0) = initContext.comZ(0) - 0.10;
  walker.applyComIK( initContext );

  // Check
  atlasSkel->setPose( initContext.state.jvalues );
  initCOM = getCOMW( atlasSkel, Twb );
  std::cout << "CoM down: "<< initCOM.transpose() << std::endl;

  if( mainDebug ) { 
    std::cout << "Init context body transform: \n" << bodyXform.matrix() << std::endl; 
    std::cout << "CoM initial: " << initContext.comX(0) <<", "<< initContext.comY(0) <<", "<<initContext.comZ(0) << std::endl; 
    std::cout << "Transformation to left leg: \n" << initContext.feet[0].matrix() << std::endl;
    std::cout << "Transformation to right leg: \n" << initContext.feet[1].matrix() << std::endl;
  }
  
  // initialize
  walker.initialize(initContext);
    
  //-- build ourselves some footprints
  Footprint initLeftFoot = Footprint( initContext.feet[0], true );

  std::vector<Footprint> footprints;
  
  if( mainDebug ) {
    std::cout <<" Walk dist: "<< walk_dist << std::endl;
    std::cout <<" Foot separation: "<< footsep_y << std::endl;
    std::cout <<" Step length: "<< step_length << std::endl;
    std::cout <<" Initial left foot position: "<< initLeftFoot.x()<<", "<<initLeftFoot.y()<<std::endl;
  }

  // Walk in a straight line
  footprints = walkLine(walk_dist, footsep_y,
			step_length,
			initLeftFoot );

  if( mainDebug ) {
    std::cout<<" Num footprints: "<<footprints.size()<<std::endl;

    // Store for debugging purposes
    FILE* fpraw;
    fpraw = fopen("footprint_raw.txt", "w");
    for( int i = 0; i < footprints.size(); ++i ) {
      fprintf( fpraw, "%d, %f %f %f \n", i, footprints[i].x(), footprints[i].y(), footprints[i].theta() );
    }
    fclose( fpraw );
  }


  //-- and then build up the walker

  // Start wait time
  walker.stayDogStay(startup_time * TRAJ_FREQ_HZ);

  // Foot steps
  for(std::vector<Footprint>::iterator it = footprints.begin(); it != footprints.end(); it++) {
    walker.addFootstep(*it);
  }
  // Close wait time
  walker.stayDogStay(shutdown_time * TRAJ_FREQ_HZ);

  
  if( mainDebug ) {
    std::cout<<" Size ref for walkLine: "<<walker.ref.size()<<std::endl;
    
    FILE* zmp; FILE* leftFoot; FILE* rightFoot; FILE* com; 
    zmp = fopen( "zmp.txt", "w" );
    leftFoot = fopen( "leftFoot.txt", "w" );
    rightFoot = fopen( "rightFoot.txt", "w" );
    com = fopen( "com.txt", "w" );
    for( int i = 0; i < walker.ref.size(); ++i ) {
      fprintf( zmp, "%d  %f %f \n", i, walker.ref[i].pX, walker.ref[i].pY );
      Eigen::Vector3d lf = walker.ref[i].feet[0].matrix().block(0,3,3,1);
      Eigen::Vector3d rf = walker.ref[i].feet[1].matrix().block(0,3,3,1);
      fprintf( leftFoot, "%d %f %f %f %d \n", i, lf(0), lf(1), lf(2), walker.ref[i].stance );
      fprintf( rightFoot, "%d %f %f %f \n", i, rf(0), rf(1), rf(2) );
      
    }

    fclose( zmp );
    fclose( leftFoot );
    fclose( rightFoot );
    fclose( com );
  }


  // Feet positions at init
  //Tw_leftFoot = atlasKin.getLimbTransW( atlasSkel, Twb, robot::robot_kinematics_t::MANIP_L_FOOT );
  //Tw_rightFoot = atlasKin.getLimbTransW( atlasSkel, Twb, robot::robot_kinematics_t::MANIP_R_FOOT );

  
  //-- have the walker run preview control and pass on the output
  //walker.bakeIt();
  
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
 * @function getCOMW
 */
Eigen::Vector3d getCOMW(kinematics::Skeleton *_atlas, const Eigen::Matrix4d &_Twb) {

	Eigen::Vector4d v4d;
	v4d.head(3) = _atlas->getWorldCOM();
	v4d(3) = 1;
	return (_Twb * v4d).head(3);

}
