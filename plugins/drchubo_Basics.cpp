/**
 * @file drchubo_Basics.cpp
 * @author A. Huaman
 */
#include "drchubo_Basics.h"

namespace gazebo {

  /**
   * @function drchubo_Basics
   * @brief Constructor
   */
  drchubo_Basics::drchubo_Basics() {
  }

  /**
   * @function ~drchubo_Basics
   * @brief Destructor
   */  
  drchubo_Basics::~drchubo_Basics() {

    event::Events::DisconnectWorldUpdateBegin( this->mUpdateConnection ); 
  }
  
  /**
   * @function Load
   * @brief
   */
  void drchubo_Basics::Load( physics::ModelPtr _parent, 
				sdf::ElementPtr _sdf ) {
    printf("Load from drchubo_Basics \n");
    this->mModel = _parent;
    this->mWorld = this->mModel->GetWorld();


    SetHardCodedInitialPose();

    // Set to update every world cycle. Listen to update event
    this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin( boost::bind(&drchubo_Basics::UpdateStates, this));
    
  }
  
  /**
   * @function SetHardCodedInitialPose
   * @brief
   */
  void drchubo_Basics::SetHardCodedInitialPose() {
    

    std::map<std::string, double> joint_position_map;

    double deg2rad = 3.1416/180.0;
    // Left Leg
    joint_position_map["drchubo::LHY"] = 0.0;
    joint_position_map["drchubo::LHR"] = 0.0; 
    joint_position_map["drchubo::LHP"] = -45*deg2rad;
    joint_position_map["drchubo::LKP"] = 90.0*deg2rad;
    joint_position_map["drchubo::LAP"] = -45.0*deg2rad;
    joint_position_map["drchubo::LAR"] = 0.0; 

    // Right Leg
    joint_position_map["drchubo::RHY"] = 0.0;
    joint_position_map["drchubo::RHR"] = 0.0; 
    joint_position_map["drchubo::RHP"] = -45.0*deg2rad;
    joint_position_map["drchubo::RKP"] = 90.0*deg2rad;
    joint_position_map["drchubo::RAP"] = -45.0*deg2rad;
    joint_position_map["drchubo::RAR"] = 0.0;


    // Torso
    joint_position_map["drchubo::TSY"] = 0.0;
    // Neck
    joint_position_map["drchubo::NKY"] = 0.0;
    joint_position_map["drchubo::NKP"] = 0.0;

    // Left Arm
    joint_position_map["drchubo::LSP"] = 0.0;
    joint_position_map["drchubo::LSR"] = 0.0;
    joint_position_map["drchubo::LSY"] = 0.0;
    joint_position_map["drchubo::LEP"] = 0.0;
    joint_position_map["drchubo::LWY"] = 0.0;
    joint_position_map["drchubo::LWP"] = 0.0;
    joint_position_map["drchubo::LWR"] = 0.0;

    // Right Arm
    joint_position_map["drchubo::RSP"] = 0.0;
    joint_position_map["drchubo::RSR"] = 0.0;
    joint_position_map["drchubo::RSY"] = 0.0;
    joint_position_map["drchubo::REP"] = 0.0;
    joint_position_map["drchubo::RWY"] = 0.0;
    joint_position_map["drchubo::RWP"] = 0.0;
    joint_position_map["drchubo::RWR"] = 0.0;

    // Left Hand
    joint_position_map["drchubo::LF1"] = 0.0;
    joint_position_map["drchubo::LF2"] = 0.0;
    joint_position_map["drchubo::LF3"] = 0.0;

    // Right Hand
    joint_position_map["drchubo::RF1"] = 0.0;
    joint_position_map["drchubo::RF2"] = 0.0;
    joint_position_map["drchubo::RF3"] = 0.0;

    this->mModel->SetJointPositions( joint_position_map );

    /*
    int count = 0;
    for( physics::Joint_V::const_iterator j = this->mModel->GetJoints().begin();
	 j != this->mModel->GetJoints().end(); ++j ) {
      (*j)->SetAngle(0, 0.5);
      count++;
    }
    printf("Set joints %d times \n", count);
    */

    printf("Setted ! \n");

  }

  /**
   * @function UpdateStates
   * @brief
   */
  void drchubo_Basics::UpdateStates() {
    
    common::Time cur_time = this->mWorld->GetSimTime();
    /*
    bool is_paused = this->mWorld->IsPaused();
    if( !is_paused ) this->mWorld->SetPaused(true);

    std::map<std::string, double> joint_position_map;

    joint_position_map["drchubo::LSR"] = 0.3;
    joint_position_map["drchubo::RSR"] = -0.3;
    joint_position_map["drchubo::LSP"] = 0.4;
    joint_position_map["drchubo::RSP"] = 0.4;

    joint_position_map["drchubo::LHP"] = -10*3.1416/180.0;
    joint_position_map["drchubo::RHP"] = -10*3.1416/180.0;
    joint_position_map["drchubo::LKP"] =  20*3.1416/180.0;
    joint_position_map["drchubo::RKP"] =  20*3.1416/180.0;
    joint_position_map["drchubo::LAP"] = -10*3.1416/180.0;
    joint_position_map["drchubo::RAP"] = -10*3.1416/180.0;

    joint_position_map["drchubo::LHY"] = 0.0;
    joint_position_map["drchubo::RHY"] = 0.0;
    joint_position_map["drchubo::LHR"] = 0.0;
    joint_position_map["drchubo::RHR"] = 0.0;
    joint_position_map["drchubo::LAR"] = 0.0;
    joint_position_map["drchubo::RAR"] = 0.0;

    this->mModel->SetJointPositions( joint_position_map );

    // Resume original pause-state
    this->mWorld->SetPaused( is_paused );
    */
  }
  
  /**
   * @function FixLink
   * @brief
   */
  void drchubo_Basics::FixLink( physics::LinkPtr _link ) {
    
    this->mJoint = this->mWorld->GetPhysicsEngine()->CreateJoint( "revolute", this->mModel );
    this->mJoint->SetModel( this->mModel );
    math::Pose pose = _link->GetWorldPose();
    this->mJoint->Load( physics::LinkPtr(), _link, pose );
    this->mJoint->SetAxis( 0, math::Vector3(0,0,0) );
    this->mJoint->SetHighStop(0,0);
    this->mJoint->SetLowStop(0,0);
    this->mJoint->SetAnchor(0, pose.pos );
    this->mJoint->Init();
  }
  
  /**
   * @function UnfixLink
   * @brief
   */
  void drchubo_Basics::UnfixLink() {
    this->mJoint.reset();
  }
    

  GZ_REGISTER_MODEL_PLUGIN( drchubo_Basics )
} // end namespace gazebo
