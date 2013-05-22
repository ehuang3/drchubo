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

    // Set actuated joints
    mNumActuatedJoints = 35;
    mActuatedJointNames.resize( mNumActuatedJoints );

    // Left Leg
    mActuatedJointNames[0]="LHY";
    mActuatedJointNames[1]="LHR"; 
    mActuatedJointNames[2]="LHP";
    mActuatedJointNames[3]="LKP";
    mActuatedJointNames[4]="LAP";
    mActuatedJointNames[5]="LAR"; 

    // Right Leg
    mActuatedJointNames[6]="RHY";
    mActuatedJointNames[7]="RHR"; 
    mActuatedJointNames[8]="RHP";
    mActuatedJointNames[9]="RKP";
    mActuatedJointNames[10]="RAP";
    mActuatedJointNames[11]="RAR";

    // Left Arm
    mActuatedJointNames[12]="LSP";
    mActuatedJointNames[13]="LSR";
    mActuatedJointNames[14]="LSY";
    mActuatedJointNames[15]="LEP";
    mActuatedJointNames[16]="LWY";
    mActuatedJointNames[17]="LWP";
    mActuatedJointNames[18]="LWR";

    // Right Arm
    mActuatedJointNames[19]="RSP";
    mActuatedJointNames[20]="RSR";
    mActuatedJointNames[21]="RSY";
    mActuatedJointNames[22]="REP";
    mActuatedJointNames[23]="RWY";
    mActuatedJointNames[24]="RWP";
    mActuatedJointNames[25]="RWR";

    // Left Hand
    mActuatedJointNames[26]="LF1";
    mActuatedJointNames[27]="LF2";
    mActuatedJointNames[28]="LF3";

    // Right Hand
    mActuatedJointNames[29]="RF1";
    mActuatedJointNames[30]="RF2";
    mActuatedJointNames[31]="RF3";

    // Torso
    mActuatedJointNames[32]="TSY";
    // Neck
    mActuatedJointNames[33]="NKY";
    mActuatedJointNames[34]="NKP";

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

    // Fill the actuated joints
    mActuatedJoints.resize( mNumActuatedJoints );
    for( int i = 0; i < mNumActuatedJoints; ++i ) {
      mActuatedJoints[i] = mModel->GetJoint( mActuatedJointNames[i].c_str() );
    }

    // Set initial hard-coded pose
    SetHardCodedInitialPose();

    // Initialize control values
    SetInitControl();

    // Save last update time
    mLastUpdatedTime = mModel->GetWorld()->GetSimTime();

    // Set to update every world cycle. Listen to update event
    this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin( boost::bind(&drchubo_Basics::UpdateStates, this));
    
  }

  /**
   * @function setInitControl
   */
  void drchubo_Basics::SetInitControl() {

    // Set info to controller
    mCb.setSize( mNumActuatedJoints );    
    mCb.setJoints( mActuatedJoints );

    // Set target to keep : Initial hard-coded values
    std::vector<double> targets( mNumActuatedJoints );
    for( int i = 0; i < mNumActuatedJoints; ++i ) {
      targets[i] = mActuatedJoints[i]->GetAngle(0).Radian(); 
    }  
    mCb.setTargets(targets);
  
    // Set PID initial values

    // Left Leg
    mCb.initPID( 0, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 1, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 2, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 3, 1000, 0, 5, 0, 0, 1000, -1000 ); // LKP
    mCb.initPID( 4, 200, 0, 10, 0, 0, 200, -200 ); // LAP
    mCb.initPID( 5, 50, 0, 5, 0, 0, 50, -50 );

    // Right Leg
    mCb.initPID( 6, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 7, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 8, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 9, 1000, 0, 5, 0, 0, 1000, -1000 ); // RKP
    mCb.initPID( 10, 200, 0, 10, 0, 0, 200, -200 ); // RAP
    mCb.initPID( 11, 50, 0, 5, 0, 0, 50, -50 );
    
    // Left Arm
    mCb.initPID( 12, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 13, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 14, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 15, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 16, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 17, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 18, 50, 0, 0, 0, 0, 50, -50 );

    // Right Arm
    mCb.initPID( 19, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 20, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 21, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 22, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 23, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 24, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 25, 50, 0, 0, 0, 0, 50, -50 );

    // Torso and neck
    mCb.initPID( 26, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 27, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 28, 50, 0, 0, 0, 0, 50, -50 );


    // Left and right fingers ( 3 + 3 = 6 )
    mCb.initPID( 29, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 30, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 31, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 32, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 33, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 34, 50, 0, 0, 0, 0, 50, -50 );

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

    // Get current time and dt
    common::Time current_time = this->mWorld->GetSimTime();
    double dt = current_time.Double() - mLastUpdatedTime.Double();

    // Call control update
    mCb.updateControls( dt );

    // Update last time
    mLastUpdatedTime = current_time;
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
