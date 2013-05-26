/**
 * @file drchubo_Animation.cpp
 */
#ifndef _DRCHUBO_MATT_CODE_
#define _DRCHUBO_MATT_CODE_

#include <map>
#include "gazebo.hh"
#include "common/common.hh"
#include "physics/physics.hh"
#include "../../src/utils/data_paths.h"

using namespace fakerave;

typedef std::vector< zmp_traj_element_t > TrajVector;

size_t seconds_to_ticks(double s) {
  return size_t(round(s*TRAJ_FREQ_HZ));
}

const int stance_foot_table[4] = { 0, 1, 0, 1 };
const int swing_foot_table[4] = { -1, -1, 1, 0 };

const stance_t next_stance_table[4] = {
  SINGLE_LEFT,
  SINGLE_RIGHT,
  DOUBLE_RIGHT,
  DOUBLE_LEFT
};

enum walktype {
  walk_canned,
  walk_line,
  walk_circle
};

namespace gazebo
{
  /**
   * @class drchubo_MattCode
   */
  class drchubo_MattCode : public ModelPlugin
  {
  public:
    drchubo_MattCode();
    ~drchubo_MattCode();
    void Load( physics::ModelPtr _model, 
	       sdf::ElementPtr _sdf );
    
    ///////////////////////////////////////
    // Matt's code functions
    void generateZMPGait(); 

    // Helpers
    double getdouble(const char* str);
    long getlong(const char* str);
    walktype getwalktype(const std::string& s); 
    ZMPWalkGenerator::ik_error_sensitivity getiksense(const std::string& s);
    Eigen::Matrix4d tf2Mx( Transform3 _tf );
    ///////////////////////////////////////

  private:
    //void UpdateStates();
    //void FixLink( physics::LinkPtr _link );
    //void UnfixLink();

    // Variables for matt's parsing types
    static const int mNumBodyDofs;
    static const int mNumJoints;
    static std::string mJointNames[];
    std::vector< Eigen::VectorXd > mMzJointTraj;
    
    physics::WorldPtr mWorld;
    physics::ModelPtr mModel;
    physics::JointPtr mJoint;

    boost::mutex mUpdate_Mutex;


    common::Time mLastUpdatedTime;


    // Pointer to the update event connection
    event::ConnectionPtr mUpdateConnection;

  };

}

#endif /** _DRCHUBO_MATT_CODE_  */ 
