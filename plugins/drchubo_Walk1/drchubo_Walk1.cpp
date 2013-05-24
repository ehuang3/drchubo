/**
 * @file drchubo_Walk1.cpp
 * @author A. Huaman
 */
#include "drchubo_Walk1.h"

std::string mWholeJointNames[26] = {"drchubo::LSP", "drchubo::LSR", "drchubo::LSY", "drchubo::LEP", "drchubo::LWY", "drchubo::LWP", "drchubo::LWR",
				    "drchubo::RSP", "drchubo::RSR", "drchubo::RSY", "drchubo::REP", "drchubo::RWY", "drchubo::RWP", "drchubo::RWR",
				    "drchubo::LHY", "drchubo::LHR", "drchubo::LHP", "drchubo::LKP", "drchubo::LAP", "drchubo::LAR",
				    "drchubo::RHY", "drchubo::RHR", "drchubo::RHP", "drchubo::RKP", "drchubo::RAP", "drchubo::RAR"};


// Other default stuff
double mKp_atlas[26] = { 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0, 5.0, // These last is for the extra DOF in the arm
			 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0, 5.0, // same here
			 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0,
			 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0  };

double mKi_atlas[26] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
			 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
			 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
			 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  

double mKd_atlas[26] = { 3.0, 20.0, 3.0, 3.0, 0.1, 0.2, 0.0, // These last is for the extra DOF in the arm
			 3.0, 20.0, 3.0, 3.0, 0.1, 0.2, 0.0, // same here
			 0.01, 1.0, 10.0, 10.0, 2.0, 1.0, 
			 0.01, 1.0, 10.0, 10.0, 2.0, 1.0 };

double mKp_def[26] = { 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 
		       50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 
		       50.0, 50.0, 50.0, 1000.0, 200.0, 50.0,
		       50.0, 50.0, 50.0, 1000.0, 200.0, 50.0 };

double mKi_def[26] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		       0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

double mKd_def[26] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		       5.0, 5.0, 5.0, 5.0, 10.0, 5.0, 
		       5.0, 5.0, 5.0, 5.0, 10.0, 5.0 };

namespace gazebo {

  /**
   * @function drchubo_Walk1
   * @brief Constructor
   */
  drchubo_Walk1::drchubo_Walk1() {

    // Set actuated joints
    mNumActuatedJoints = 35;
    mNumReadJoints = 26;
    mTrajCounter = 0;
    mUpdateCounter = 0;
    mActuatedJointNames.resize( mNumActuatedJoints );
    
    // Left Arm
    mActuatedJointNames[0]="LSP";
    mActuatedJointNames[1]="LSR";
    mActuatedJointNames[2]="LSY";
    mActuatedJointNames[3]="LEP";
    mActuatedJointNames[4]="LWY";
    mActuatedJointNames[5]="LWP";
    mActuatedJointNames[6]="LWR";

    // Right Arm
    mActuatedJointNames[7]="RSP";
    mActuatedJointNames[8]="RSR";
    mActuatedJointNames[9]="RSY";
    mActuatedJointNames[10]="REP";
    mActuatedJointNames[11]="RWY";
    mActuatedJointNames[12]="RWP";
    mActuatedJointNames[13]="RWR";

    // Left Leg
    mActuatedJointNames[14]="LHY";
    mActuatedJointNames[15]="LHR"; 
    mActuatedJointNames[16]="LHP";
    mActuatedJointNames[17]="LKP";
    mActuatedJointNames[18]="LAP";
    mActuatedJointNames[19]="LAR"; 

    // Right Leg
    mActuatedJointNames[20]="RHY";
    mActuatedJointNames[21]="RHR"; 
    mActuatedJointNames[22]="RHP";
    mActuatedJointNames[23]="RKP";
    mActuatedJointNames[24]="RAP";
    mActuatedJointNames[25]="RAR";

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
   * @function ~drchubo_Walk1
   * @brief Destructor
   */  
  drchubo_Walk1::~drchubo_Walk1() {

    event::Events::DisconnectWorldUpdateBegin( this->mUpdateConnection ); 
  }
  
  /**
   * @function Load
   * @brief
   */
  void drchubo_Walk1::Load( physics::ModelPtr _parent, 
			    sdf::ElementPtr _sdf ) {
    printf("Calling Load Function \n");
    this->mModel = _parent;
    this->mWorld = this->mModel->GetWorld();
    
    // Fill the actuated joints
    mActuatedJoints.resize( mNumActuatedJoints );
    for( int i = 0; i < mNumActuatedJoints; ++i ) {
      mActuatedJoints[i] = mModel->GetJoint( mActuatedJointNames[i].c_str() );
    }
    
    // Read Trajectory file
    ReadTrajectoryFile();
    
    // Set initial hard-coded pose
    SetHardCodedInitialPose();
    
    // Initialize control values
    SetInitControl();
    
    // Save last update time
    mLastUpdatedTime = mModel->GetWorld()->GetSimTime();
    
    // Set to update every world cycle. Listen to update event
    this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin( boost::bind(&drchubo_Walk1::UpdateStates, this));
    
  }
  
  /**
   * @function ReadTrajectoryFile
   */
  void drchubo_Walk1::ReadTrajectoryFile() {

    float ang[mNumReadJoints]; int ind;
    int numTrajPoints = 900;

    // Reset
    mTrajPoints.resize(0);
    mTrajStance.resize(0);

    // Read file with trajectories and stance
    FILE* pFile; FILE* sFile;
    pFile = fopen("/home/ana/Research/drchubo/data/trajs/traj.txt", "r");
    sFile = fopen("/home/ana/Research/drchubo/data/trajs/stance.txt", "r");
    
    if( pFile == NULL || sFile == NULL ) {
      printf("Did not find either traj or stance file, exiting! \n");
      return;
    }
    
    for( int i = 0; i < numTrajPoints; ++i ) {
      // Left Arm
      fscanf( pFile, "%d %f %f %f %f %f %f", &ind, &ang[0], &ang[1], &ang[2], &ang[3], &ang[4], &ang[5] ); 
      ang[6] = 0.0;
      // Right Arm
      fscanf( pFile, "%f %f %f %f %f %f", &ang[7], &ang[8], &ang[9], &ang[10], &ang[11], &ang[12] );
      ang[13] = 0.0;
      // Left Leg
      fscanf( pFile, "%f %f %f %f %f %f", &ang[14], &ang[15], &ang[16], &ang[17], &ang[18], &ang[19] );
      // Right Leg
      fscanf( pFile, "%f %f %f %f %f %f \n", &ang[20], &ang[21], &ang[22], &ang[23], &ang[24], &ang[25] );
      
      // Read each traj point (leave as 0.0 the rest of them)
      std::vector<double> trajPoint( mNumActuatedJoints, 0.0  );
      for( int j = 0; j < mNumReadJoints; ++j ) {
	trajPoint[j] = ang[j];
      }
      // Read stance
      int stance;
      fscanf( sFile, "%d \n", &stance );
      
      // Save them
      mTrajPoints.push_back( trajPoint );
      mTrajStance.push_back( stance );
    }
    
    fclose( pFile );
    fclose( sFile );
    
    printf("Read trajectory and stance file \n");
    
  }
  
  /**
   * @function setInitControl
   */
  void drchubo_Walk1::SetInitControl() {
    
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
    for( int i = 0; i < mNumReadJoints; ++i ) {
      mCb.initPID( i, mKp_def[i], mKi_def[i], mKd_def[i], 0, 0, mKp_def[i], -1*mKp_def[i] );
    }

    /*   
    // Left Arm
    mCb.initPID( 0, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 1, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 2, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 3, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 4, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 5, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 6, 50, 0, 0, 0, 0, 50, -50 );

    // Right Arm
    mCb.initPID( 7, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 8, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 9, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 10, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 11, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 12, 50, 0, 0, 0, 0, 50, -50 );
    mCb.initPID( 13, 50, 0, 0, 0, 0, 50, -50 );

    // Left Leg
    mCb.initPID( 14, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 15, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 16, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 17, 1000, 0, 5, 0, 0, 1000, -1000 ); // LKP
    mCb.initPID( 18, 200, 0, 10, 0, 0, 200, -200 ); // LAP
    mCb.initPID( 19, 50, 0, 5, 0, 0, 50, -50 );

    // Right Leg
    mCb.initPID( 20, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 21, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 22, 50, 0, 5, 0, 0, 50, -50 );
    mCb.initPID( 23, 1000, 0, 5, 0, 0, 1000, -1000 ); // RKP
    mCb.initPID( 24, 200, 0, 10, 0, 0, 200, -200 ); // RAP
    mCb.initPID( 25, 50, 0, 5, 0, 0, 50, -50 );
    */

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
	
    printf("Set init Control \n");
  }
  
  /**
   * @function SetHardCodedInitialPose
   * @brief
   */
  void drchubo_Walk1::SetHardCodedInitialPose() {
    
    
    std::map<std::string, double> joint_position_map;

    // Set initial location as first trajectory point
    for( int i = 0; i < mNumReadJoints; ++i ) {
      joint_position_map[ mWholeJointNames[i] ] = mTrajPoints[0][i];
    }

    // Set remaining 
    // Torso
    joint_position_map["drchubo::TSY"] = 0.0;
    // Neck
    joint_position_map["drchubo::NKY"] = 0.0;
    joint_position_map["drchubo::NKP"] = 0.0;

    // Left Hand
    joint_position_map["drchubo::LF1"] = 0.0;
    joint_position_map["drchubo::LF2"] = 0.0;
    joint_position_map["drchubo::LF3"] = 0.0;

    // Right Hand
    joint_position_map["drchubo::RF1"] = 0.0;
    joint_position_map["drchubo::RF2"] = 0.0;
    joint_position_map["drchubo::RF3"] = 0.0;

    this->mModel->SetJointPositions( joint_position_map );


    printf("Set initial joint positions ! \n");

  }

  /**
   * @function UpdateStates
   * @brief
   */
  void drchubo_Walk1::UpdateStates() {

    // Get current time and dt
    common::Time current_time = this->mWorld->GetSimTime();
    double dt = current_time.Double() - mLastUpdatedTime.Double();
    
    mUpdateCounter++;
    
    if( mUpdateCounter % 5 == 0 ) {
      mTrajCounter++;
    }
    
    // Set Targets
    std::vector<double> targets( mNumActuatedJoints, 0.0 );
    for( int j = 0; j < mNumReadJoints; ++j ) {
      targets[j] = mTrajPoints[mTrajCounter][j]; 
    }  
    mCb.setTargets(targets);
    
    // Set gains appropriately
    setControlGains( mTrajStance[mTrajCounter] );
	
    // Call control update
    mCb.updateControls( dt );
    
    // Update last time
    mLastUpdatedTime = current_time;
  }
  

  /**
   * @function setControlGains
   */
  void drchubo_Walk1::setControlGains( int _mode ) {
    
    if( mTrajStance[mTrajCounter] == DOUBLE_LEFT || 
	mTrajStance[mTrajCounter] == DOUBLE_RIGHT ) {
      printf("DOUBLE STANCE \n");
    }
    else if( mTrajStance[mTrajCounter] == SINGLE_LEFT ) {
      printf("SINGLE LEFT \n");
    }
    else if( mTrajStance[mTrajCounter] == SINGLE_RIGHT ) {
      printf("SINGLE RIGHT \n");
    }

  }

  /**
   * @function FixLink
   * @brief
   */
  void drchubo_Walk1::FixLink( physics::LinkPtr _link ) {
    
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
  void drchubo_Walk1::UnfixLink() {
    this->mJoint.reset();
  }
    

  GZ_REGISTER_MODEL_PLUGIN( drchubo_Walk1 )
} // end namespace gazebo
