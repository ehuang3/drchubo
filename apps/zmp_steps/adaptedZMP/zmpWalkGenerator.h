/**
 * @file zmpWalkGenerator.h
 * @author A. Huaman
 */

#ifndef _ATLAS_ZMP_WALK_GENERATOR_
#define _ATLAS_ZMP_WALK_GENERATOR_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atlas/atlas_kinematics.h>
#include <robot/robot_kinematics.h>
#include <zmp/ZmpCommand.h>

#include <kinematics/Skeleton.h>
#include "AtlasKinematics_Extra.h"

#include "footprint.h"
#include "atlas_zmp.h"

/** 
 * @class ZMPReferenceContext
 */
class ZMPReferenceContext {

 public:

  stance_t stance;
  robot::robot_kinematics_t::IK_Mode ikMode[4];

  Eigen::Matrix4d feet[2];
  Eigen::Vector3d comX, comY, comZ;
  double eX, eY;
  double pX, pY;

  KState state;
  
};

/**
 * @class ZMPWalkGenerator
 */
class ZMPWalkGenerator {
 public:
  
  ZMPWalkGenerator( atlas::atlas_kinematics_t& _atlasKin,
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
		    double _lookahead_time );
  
  atlas::atlas_kinematics_t mAtlasKin;
  kinematics::Skeleton* mAtlasSkel;
  
  zmp::ik_error_sensitivity ik_sense;
  double com_height;
  double zmp_R;  /**< jerk penalty on ZMP controller */
  double zmpoff_x;
  double zmpoff_y;
  double com_ik_angle_weight;
  double min_single_support_time;
  double min_double_support_time;
  double walk_startup_time;
  double walk_shutdown_time;
  double step_height;
  double lookahead_time;

  double mFootAnkleDist;
  
  size_t first_step_index;
  
  bool haveInitContext;
  ZMPReferenceContext initContext; 
  std::vector<ZMPReferenceContext> ref;
  
  std::vector<zmp_traj_element_t> traj; /**< the entire fullbody trajectory so far */
  
  // INVARIANT: traj.back() agrees 100% with current
  
    
  // these all modify the current context but do not immediately affect traj
  void initialize( const ZMPReferenceContext &_current );
  const ZMPReferenceContext& getLastRef();
  // these will add walk contexts to the back of ref and the new
  // contexts don't have comX, comY, eX, eY however, the kstate will
  // have body orientation set correctly and upper body joints
  void stayDogStay(size_t stay_ticks);
  void addFootstep(const Footprint& fp);
  void bakeIt();
  
  void applyComIK(ZMPReferenceContext& ref);
  void dart2GazeboAngles( const Eigen::VectorXd &_dartAngles,
			  double _trajAngles[] );
  void refToTraj(const ZMPReferenceContext& ref,
		 zmp_traj_element_t& traj);
  
  
  
  // helper
  double sigmoid(double x);
  
  // this runs the ZMP preview controller on the entire reference
  // trajectory to fill in comX, comY, eX, eY for every dang thing.
  void runZMPPreview();
  // this runs the COM IK on every dang thing in reference to fill
  // in the kstate
  
  void runCOMIK();
  // this dumps everything into traj
  void dumpTraj();

  void printDebugInfo();
};

#endif /** _ATLAS_ZMP_WALK_GENERATOR_  */
