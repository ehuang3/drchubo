/**
 * @file drchubo_Walk1.h
 * @author A. Huaman
 */

#ifndef _DRCHUBO_BASICS_H_
#define _DRCHUBO_BASICS_H_


#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

// Simple PID control
#include "controls/controlBundle.h"

// Other stuff
#include <vector>

enum stance_t {
  DOUBLE_LEFT  = 0,
  DOUBLE_RIGHT = 1,
  SINGLE_LEFT  = 2,
  SINGLE_RIGHT = 3,
};

namespace gazebo {

  /**
   * @class drchubo_Walk1
   */
  class drchubo_Walk1 : public ModelPlugin
  {
  public:
    drchubo_Walk1();
    ~drchubo_Walk1();
    
    void Load( physics::ModelPtr _parent, 
	       sdf::ElementPtr _sdf );

    void SetHardCodedInitialPose();
    void SetInitControl();
    void ReadTrajectoryFile();
    void setControlGains( int _mode ); 

  private:
    void UpdateStates();
    void FixLink( physics::LinkPtr _link );
    void UnfixLink();
    
    physics::WorldPtr mWorld;
    physics::ModelPtr mModel;
    physics::JointPtr mJoint;

    boost::mutex mUpdate_Mutex;

    // Trajectories and stance storage
    std::vector< std::vector<double> > mTrajPoints;
    std::vector< int > mTrajStance;

    // Control variables
    int mNumActuatedJoints;
    int mNumReadJoints;
    int mTrajCounter;
    int mUpdateCounter;
    std::vector<std::string> mActuatedJointNames;
    std::vector<physics::JointPtr> mActuatedJoints;

    controlBundle mCb;

    common::Time mLastUpdatedTime;


    // Pointer to the update event connection
    event::ConnectionPtr mUpdateConnection;
  };

}

#endif /** _DRCHUBO_BASICS_H_ */
