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
atlas::atlas_kinematics_t atlasKin;
dynamics::SkeletonDynamics* atlasSkel;

// Function declaration
void prepareAtlasKinematics();
Eigen::VectorXd getManipAngles( kinematics::Skeleton* _atlasSkel,
				atlas::atlas_kinematics_t _ak,
				int _limb );
ZMPReferenceContext getContextDown( double _offset,
				    kinematics::Skeleton* _atlasSkel,
				    atlas::atlas_kinematics_t _ak );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  //-- Initialize atlasKin  
  prepareAtlasKinematics();

  //-- Get a context with a CoM 16 cm below the zero position
  ZMPReferenceContext initContext;
  initContext = getContextDown( -0.16,
				atlasSkel,
				atlasKin );

  /**< Get foot separation and height of CoM from Ankle */
  double footsep_y = ( initContext.feet[0].matrix()(1,3) - initContext.feet[1].matrix()(1,3) ) / 2.0;
  double com_height = initContext.comZ(0) - initContext.feet[0].matrix()(2,3);

  double foot_liftoff_z = 0.05; // foot liftoff height
  double step_length = 0.3;

  bool walk_sideways = false;  
  walktype walk_type = walk_canned;
  double walk_circle_radius = 5.0;
  double walk_dist = 5;

  double com_ik_ascl = 0;
  
  double zmpoff_y = 0;
  double zmpoff_x = 0;
  
  double lookahead_time = 2.5;
  
  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.15;
  double single_support_time = 0.85;
  
  size_t max_step_count = 25;
  
  double zmp_jerk_penalty = 1e-8; 
  
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
  
  // initialize
  walker.initialize(initContext);

  if( mainDebug ) { 
    std::cout << "\n Init body transform: \n" << initContext.state.xform().matrix() << std::endl; 
    std::cout << "CoM initial: " << initContext.comX(0) <<", "<< initContext.comY(0) <<", "<<initContext.comZ(0) << std::endl; 
    std::cout << "Transformation to left leg: \n" << initContext.feet[0].matrix() << std::endl;
    std::cout << "Transformation to right leg: \n" << initContext.feet[1].matrix() << std::endl;
  }
    
  //-- build ourselves some footprints
  Footprint initLeftFoot = Footprint( initContext.feet[0], true );
  std::cout<< "Init context foot 0: \n"<< initContext.feet[0] << std::endl;					      
  std::vector<Footprint> footprints;
  
  if( mainDebug ) {
    std::cout <<" Walk dist: "<< walk_dist << std::endl;
    std::cout <<" Foot separation: "<< footsep_y << std::endl;
    std::cout <<" Step length: "<< step_length << std::endl;
    std::cout <<" Initial left foot position: \n"<< initLeftFoot.x()<<", "<<initLeftFoot.y()<<std::endl;
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

  //-- have the walker run preview control and pass on the output
  //walker.bakeIt();
  
  return 0;
  
}

/**
 * @function getManipAngles
 */
Eigen::VectorXd getManipAngles( kinematics::Skeleton* _atlasSkel,
				atlas::atlas_kinematics_t _ak,
				int _limb ) {
  
  // Set the angles
  int nDofsNum = _atlasSkel->getNumDofs();
  Eigen::VectorXd dofs( nDofsNum );
  dofs = _atlasSkel->getPose();
  
  Eigen::VectorXd angles(6);

  for (int i = 0; i < 6; i++) {
    angles(i) = dofs( _ak.dart_dof_ind[_limb][i] );
  }
  
  return angles;
}

/**
 * @function prepareAtlasKinematics
 */
void prepareAtlasKinematics() {

  DartLoader dart_loader;
  simulation::World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH "models/atlas/atlas_world.urdf");
  atlasSkel = mWorld->getSkeleton("atlas");

  if( !atlasSkel ) {
    std::cout<<" Atlas Skeleton no loaded, exiting!"<< std::endl;
    return;
  }

  atlasKin.init( atlasSkel );
       
  atlasSkel->setPose( atlasSkel->getPose().setZero(), true );
}

/**
 * @function initContext
 */
ZMPReferenceContext getContextDown( double _offset,
				    kinematics::Skeleton* _atlasSkel,
				    atlas::atlas_kinematics_t _ak ) {
  ZMPReferenceContext initContext;

  // Set initial pose to zero and with the arms low
  Eigen::VectorXd dofs = atlasSkel->getPose().setZero();
  //dofs( 20 ) = -75.0*3.14/180.0; dofs( 22 ) = 75.0*3.14/180.0; // LSR / RSR

  /**< Transformations to use */
  Eigen::Matrix4d Twb; /**< Transformation from World to Body Frame */
  Eigen::Matrix4d Twl; /**< Transformation from World to Left Foot - DH */
  Eigen::Matrix4d Twr; /**< Transformation from World to Right Leg - DH */
  Eigen::VectorXd leftLeg_ang;
  Eigen::VectorXd rightLeg_ang;
  Eigen::Affine3d bodyXform;
  Eigen::Vector3d com;

  // Set the skeleton
  _atlasSkel->setPose( dofs );

  /**< Get angles for left and right legs (0 in this case) */
  leftLeg_ang = getManipAngles( _atlasSkel, _ak, robot::robot_kinematics_t::MANIP_L_FOOT );
  rightLeg_ang = getManipAngles( _atlasSkel, _ak, robot::robot_kinematics_t::MANIP_R_FOOT );

  /**< Fill transformations */
  Twb = _atlasSkel->getNode("pelvis")->getWorldTransform();  
  Twl = Twb*atlasKin.legFK( leftLeg_ang, true );
  Twr = Twb*atlasKin.legFK( rightLeg_ang, false );
  
  // init CoM and set it below
  com = _atlasSkel->getWorldCOM().block(0,0,3,1);

  std::cout<< "\n CoM before going down:  "<< com.transpose() << std::endl;
  std::cout<< " Twb before going down:  \n"<< Twb << std::endl;

  com(2) = com(2) + _offset;
  
  // Set
  Eigen::Matrix4d Twm[robot::robot_kinematics_t::NUM_MANIPULATORS];
  robot::robot_kinematics_t::IK_Mode mode[robot::robot_kinematics_t::NUM_MANIPULATORS];
  mode[robot::robot_kinematics_t::MANIP_L_FOOT] = robot::robot_kinematics_t::IK_MODE_SUPPORT;
  mode[robot::robot_kinematics_t::MANIP_R_FOOT] = robot::robot_kinematics_t::IK_MODE_SUPPORT;
  mode[robot::robot_kinematics_t::MANIP_L_HAND] = robot::robot_kinematics_t::IK_MODE_FIXED;
  mode[robot::robot_kinematics_t::MANIP_R_HAND] = robot::robot_kinematics_t::IK_MODE_FIXED;

  // Manipulator transforms
  Twm[robot::robot_kinematics_t::MANIP_L_FOOT] = Twl;
  Twm[robot::robot_kinematics_t::MANIP_R_FOOT] = Twr;
   
  // AND RUN IK. Everything we need goes straight into cur.state! cool.

  // Apply IK
  _ak.comIK( _atlasSkel,
	     com,
	     Twb,
	     mode,
	     Twm,
	     dofs );

  /** Applied IK. Now UPDATE initContext after the IK and return it */
  _atlasSkel->setPose( dofs );

  com = _atlasSkel->getWorldCOM().block(0,0,3,1);

  // Update CoM
  initContext.comX(0) = com(0);
  initContext.comY(0) = com(1);
  initContext.comZ(0) = com(2);

  // init desired ZMP
  initContext.eX = 0.0; initContext.eY = 0.0;
  initContext.pX = 0.0; initContext.pY = 0.0;  

  // Fill the kstate
  initContext.state.jvalues = dofs;

  // Get the angles for the legs
  leftLeg_ang = getManipAngles( _atlasSkel, _ak, robot::robot_kinematics_t::MANIP_L_FOOT );
  rightLeg_ang = getManipAngles( _atlasSkel, _ak, robot::robot_kinematics_t::MANIP_R_FOOT );

  // Update transformations 
  Twb = atlasSkel->getNode("pelvis")->getWorldTransform();  
  Twl = Twb*_ak.legFK( leftLeg_ang, true );
  Twr = Twb*_ak.legFK( rightLeg_ang, false );

  // init body transformation
  bodyXform.matrix() = Twb;

  initContext.state.setXform( bodyXform );
  initContext.stance = DOUBLE_LEFT;

  // Finally, set the feet
  initContext.feet[0].matrix() = Twl;
  initContext.feet[1].matrix() = Twr;

  std::cout<< " CoM after going down:  "<< com.transpose() << std::endl;
  std::cout<< " Twb after going down: \n "<< Twb << std::endl;

  return initContext;
}
