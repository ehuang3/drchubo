/**
 * @file drchubo_Basics.h
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


namespace gazebo {

  /**
   * @class drchubo_Basics
   */
  class drchubo_Basics : public ModelPlugin
  {
  public:
    drchubo_Basics();
    ~drchubo_Basics();
    
    void Load( physics::ModelPtr _parent, 
	       sdf::ElementPtr _sdf );

    void SetHardCodedInitialPose();
    void SetInitControl();

  private:
    void UpdateStates();
    void FixLink( physics::LinkPtr _link );
    void UnfixLink();
    
    physics::WorldPtr mWorld;
    physics::ModelPtr mModel;
    physics::JointPtr mJoint;

    boost::mutex mUpdate_Mutex;

    // Control variables
    int mNumActuatedJoints;
    std::vector<std::string> mActuatedJointNames;
    std::vector<physics::JointPtr> mActuatedJoints;

    controlBundle mCb;

    common::Time mLastUpdatedTime;


    // Pointer to the update event connection
    event::ConnectionPtr mUpdateConnection;
  };

}

#endif /** _DRCHUBO_BASICS_H_ */
