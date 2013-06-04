/*
 * @file DRCPlugin.cpp
 * @brief
 */

#include <map>
#include <string>
#include <stdlib.h>

#include <iostream>
#include <new>

#include "DRCPlugin.h"

// Gazebo Animation 
#include <common/common.hh>

// For callbacks we use boost::function
#include <boost/function.hpp>

#include <sys/types.h>


double mod(double _x, double _y) {
    if (0 == _y) return _x;
    return _x - _y * floor(_x/_y);
}

double clamp2pi(double _ang) {
   return mod(_ang + M_PI, 2*M_PI) - M_PI;
}


namespace gazebo
{
    GZ_REGISTER_WORLD_PLUGIN(DRCPlugin)
/*
    const int DRCPlugin::mNumJoints = 35;
    std::string DRCPlugin::mFullJointNames[] = {"drchubo::LSP", "drchubo::LSR", "drchubo::LSY", "drchubo::LEP", "drchubo::LWY", "drchubo::LWP", "drchubo::LWR",
                                                "drchubo::RSP", "drchubo::RSR", "drchubo::RSY", "drchubo::REP", "drchubo::RWY", "drchubo::RWP", "drchubo::RWR",
                                                "drchubo::LHY", "drchubo::LHR", "drchubo::LHP", "drchubo::LKP", "drchubo::LAP", "drchubo::LAR",
                                                "drchubo::RHY", "drchubo::RHR", "drchubo::RHP", "drchubo::RKP", "drchubo::RAP", "drchubo::RAR",
                                                "drchubo::TSY", "drchubo::NKY", "drchubo::NKP",
                                                "drchubo::LF1", "drchubo::LF2", "drchubo::LF3",
                                                "drchubo::RF1", "drchubo::RF2", "drchubo::RF3"};

    std::string DRCPlugin::mJointNames[] = {"LSP", "LSR", "LSY", "LEP", "LWY", "LWP", "LWR",
                                            "RSP", "RSR", "RSY", "REP", "RWY", "RWP", "RWR",
                                            "LHY", "LHR", "LHP", "LKP", "LAP", "LAR",
                                            "RHY", "RHR", "RHP", "RKP", "RAP", "RAR",
                                            "TSY", "NKY", "NKP",
                                            "LF1", "LF2", "LF3",
                                            "RF1", "RF2", "RF3"};
*/

    const int DRCPlugin::mNumJoints = 29;
    std::string DRCPlugin::mFullJointNames[] = {"drchubo::LSP", "drchubo::LSR", "drchubo::LSY", "drchubo::LEP", "drchubo::LWY", "drchubo::LWP", "drchubo::LWR",
                                                "drchubo::RSP", "drchubo::RSR", "drchubo::RSY", "drchubo::REP", "drchubo::RWY", "drchubo::RWP", "drchubo::RWR",
                                                "drchubo::LHY", "drchubo::LHR", "drchubo::LHP", "drchubo::LKP", "drchubo::LAP", "drchubo::LAR",
                                                "drchubo::RHY", "drchubo::RHR", "drchubo::RHP", "drchubo::RKP", "drchubo::RAP", "drchubo::RAR",
                                                "drchubo::TSY", "drchubo::NKY", "drchubo::NKP"};

    std::string DRCPlugin::mJointNames[] = {"LSP", "LSR", "LSY", "LEP", "LWY", "LWP", "LWR",
                                            "RSP", "RSR", "RSY", "REP", "RWY", "RWP", "RWR",
                                            "LHY", "LHR", "LHP", "LKP", "LAP", "LAR",
                                            "RHY", "RHR", "RHP", "RKP", "RAP", "RAR",
                                            "TSY", "NKY", "NKP"};


////////////////////////////////////////////////////////////////////////////////
// Constructor
    DRCPlugin::DRCPlugin()
    {
    }

////////////////////////////////////////////////////////////////////////////////
// Destructor
    DRCPlugin::~DRCPlugin()
    {
        event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
        this->rosNode->shutdown();
        this->rosQueue.clear();
        this->rosQueue.disable();
        this->callbackQueueThread.join();
        delete this->rosNode;
    }

////////////////////////////////////////////////////////////////////////////////
// Load the controller
    void DRCPlugin::Load( physics::WorldPtr _parent, 
                          sdf::ElementPtr _sdf )
    {
        // save pointers
        this->world = _parent;
        this->sdf = _sdf;
  
        // ros callback queue for processing subscription
        // this->deferredLoadThread = boost::thread(
        //   boost::bind(&VRCPlugin::DeferredLoad, this));
        this->DeferredLoad();
    }

////////////////////////////////////////////////////////////////////////////////
    // Load the controller
    void DRCPlugin::DeferredLoad()
    {
        // initialize ros
        if ( !ros::isInitialized() )
        {
            gzerr << "Not loading DRC plugin since ROS hasn't been "
                  << "properly initialized.  Try starting gazebo with ros plugin:\n"
                  << "  gazebo -s libgazebo_ros_api_plugin.so\n";
            return;
        }

        // ros stuff
        this->rosNode = new ros::NodeHandle("");

        // load DRC ROS API
        this->LoadDRCROSAPI();

        // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
        this->lastUpdateTime = this->world->GetSimTime().Double();

        // Load Robot
        this->drchubo.Load(this->world, this->sdf);

	// Load drill
	this->drill.Load( this->world, this->sdf );

        // Setup ROS interfaces for robot
        this->LoadRobotROSAPI();
    
        // Set robot mode to no_gravity to see what happens
        this->SetRobotMode( "no_gravity" );

        // Set robot parallel to floor
        this->drchubo.SetFootParallelToFloor();

        // Store the default configuration and pose of the robot
        getCurrentJointState();
        getCurrentPose();

        // Set onAnimation to false
        this->onJointAnimation = false;
        this->onPoseAnimation = false;

        // Set callbacks to boost::function form
        // to be suitable arguments for animation callback arg
        jointAnim_callback = boost::bind(&DRCPlugin::jointAnimation_callback, this );
        poseAnim_callback = boost::bind(&DRCPlugin::poseAnimation_callback, this );


        // ros callback queue for processing subscription
        this->callbackQueueThread = boost::thread(
            boost::bind(&DRCPlugin::ROSQueueThread, this));

        // Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DRCPlugin::UpdateStates, this));

        for(int i=0; i < mNumJoints; i++) {
            std::cout << drchubo.mJoints[i]->GetName() << " limits = " <<
                drchubo.mJoints[i]->GetLowerLimit(0) << ", " << drchubo.mJoints[i]->GetUpperLimit(0) << std::endl;
        }

        ROS_INFO("[DRCPlugin] End of Deferred Load \n");
    }



////////////////////////////////////////////////////////////////////////////////
void DRCPlugin::RobotGrabDrill(const geometry_msgs::Pose::ConstPtr &_cmd)
{
  math::Quaternion q(_cmd->orientation.w, _cmd->orientation.x,
                     _cmd->orientation.y, _cmd->orientation.z);
  q.Normalize();
  math::Pose pose(math::Vector3(_cmd->position.x,
                                _cmd->position.y,
                                _cmd->position.z), q);
  /// \todo: get these from incoming message
  std::string gripperName = "Body_RWR";
  math::Pose relPose(math::Vector3(0.0, 0.0, 0),
               math::Quaternion(0, 0, 0));

  if (this->drill.drillModel && this->drill.couplingLink)
  {
    // Set drillModel to no collision
    this->drill.drillModel->GetLink("link")->SetSelfCollide(false);
    this->drill.drillModel->GetLink("link")->SetCollideMode("none"); 

    physics::LinkPtr gripper = this->drchubo.model->GetLink(gripperName);
    if (gripper)
    {  printf("[DRCPlugin] Set gripper to %s \n", gripperName.c_str() );
      // teleports the object being attached together
      pose = pose + relPose + gripper->GetWorldPose();
      this->drill.drillModel->SetLinkWorldPose(pose,
        this->drill.couplingLink);

    // Disable collision of fingers
    // Should we put collision back once we let the drill go?
   this->drchubo.model->GetLink("Body_RF1")->SetSelfCollide(false); 
   this->drchubo.model->GetLink("Body_RF1")->SetCollideMode("none");
   this->drchubo.model->GetLink("Body_RF2")->SetSelfCollide(false);  
   this->drchubo.model->GetLink("Body_RF2")->SetCollideMode("none"); 
   this->drchubo.model->GetLink("Body_RF3")->SetSelfCollide(false); 
   this->drchubo.model->GetLink("Body_RF3")->SetCollideMode("none");

   this->drchubo.model->GetLink("Body_LF1")->SetSelfCollide(false); 
   this->drchubo.model->GetLink("Body_LF1")->SetCollideMode("none");
   this->drchubo.model->GetLink("Body_LF2")->SetSelfCollide(false);  
   this->drchubo.model->GetLink("Body_LF2")->SetCollideMode("none"); 
   this->drchubo.model->GetLink("Body_LF3")->SetSelfCollide(false); 
   this->drchubo.model->GetLink("Body_LF3")->SetCollideMode("none"); 

    // Set drillModel to no collision
    this->drill.drillModel->GetLink("link")->SetSelfCollide(false);
    this->drill.drillModel->GetLink("link")->SetCollideMode("none"); 

    for(int i=0; i < mNumJoints; i++) {
        std::cout << mJointNames[i] << std::endl;
        if(mJointNames[i] != "TSY") {
            this->drchubo.model->GetLink("Body_" + mJointNames[i])->SetSelfCollide(false);
            this->drchubo.model->GetLink("Body_" + mJointNames[i])->SetCollideMode("none");
        } else {
            this->drchubo.model->GetLink("Body_Torso")->SetSelfCollide(false);
            this->drchubo.model->GetLink("Body_Torso")->SetCollideMode("none");
        }
    }

    // Nothing should self collide?


      if (!this->grabJoint)
        this->grabJoint = this->AddJoint(this->world, this->drill.drillModel,
                                         gripper,
                                         this->drill.couplingLink,
                                         "revolute",
                                         math::Vector3(0, 0, 0),
                                         math::Vector3(0, 0, 1),
                                         0.0, 0.0,true);
    }

   else {
      printf("[DRCPlugin] NO Set gripper to %s \n", gripperName.c_str() );
   }

  } 
}

////////////////////////////////////////////////////////////////////////////////
void DRCPlugin::RobotReleaseLink(const geometry_msgs::Pose::ConstPtr &/*_cmd*/)
{
    if(this->grabJoint)
        this->RemoveJoint(this->grabJoint);
}

  // ******************************************
  // DRILL FUNCTIONS
void DRCPlugin::Drill::SetInitialConfiguration()
{
  // this does not work yet, because SetAngle only works for Hinge and Slider
  // joints, and fire hose is made of universal and ball joints.
  for (unsigned int i = 0; i < this->drillJoints.size(); ++i)
    {
      // gzerr << "joint [" << this->fireHoseJoints[i]->GetName() << "]\n";
      this->drillJoints[i]->SetAngle(0u, 0.0);
  }
}

  /**
   * @function DRILL Load
   */
void DRCPlugin::Drill::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->isInitialized = false;

  sdf::ElementPtr sdf = _sdf->GetElement("drill");
  // Get special coupling links (on the firehose side)
  std::string drillModelName = sdf->GetValueString("drill_model");
  this->drillModel = _world->GetModel(drillModelName);
  if (!this->drillModel)
  {
    ROS_INFO("Drill [%s] not found", drillModelName.c_str());
    return;
  }
  this->initialDrillPose = this->drillModel->GetWorldPose();

  // Get coupling link
  std::string couplingLinkName = sdf->GetValueString("coupling_link");
  this->couplingLink = this->drillModel->GetLink(couplingLinkName);
  if (!this->couplingLink)
  {
    ROS_ERROR("coupling link [%s] not found", couplingLinkName.c_str());
    return;
  }

  // Get joints
  this->drillJoints = this->drillModel->GetJoints();

  // Get links
  this->drillLinks = this->drillModel->GetLinks();

  
  //this->threadPitch = sdf->GetValueDouble("thread_pitch");

  this->couplingRelativePose = sdf->GetValuePose("coupling_relative_pose");

  // Set initial configuration
  this->SetInitialConfiguration();

  this->isInitialized = true;
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
    void DRCPlugin::UpdateStates()
    {
        std::cout << "Aquiring lock in UpdateStates" << std::endl;
        boost::mutex::scoped_lock lock(this->update_mutex);

    
        double curTime = this->world->GetSimTime().Double();
    
        // If we are animating don't do any action. Let the animation stop
        if( this->onJointAnimation == true || this->onPoseAnimation == true ) {
            return;
        }
      
        // Will try to keep at the joint configuration last sent through drc/configuration
        if( this->drchubo.modeType == ON_STAY_DOG_MODE ) {
            // Set joint pose
            drchubo.model->SetJointPositions( defaultJointState_p );

            // Set world pose   
            this->drchubo.model->SetWorldPose( defaultPose_p );

        }
      
        // **************************************************
        // PUBLISH ROBOT STATE

        // Pose
        geometry_msgs::Pose pose_msg;

        math::Pose p = this->drchubo.model->GetWorldPose();
        math::Pose lf = this->drchubo.model->GetLink("Body_LAR")->GetWorldPose();
        pose_msg.position.x = p.pos.x;
        pose_msg.position.y = p.pos.y;
        pose_msg.position.z = p.pos.z;

        pose_msg.orientation.x = p.rot.x;
        pose_msg.orientation.y = p.rot.y;
        pose_msg.orientation.z = p.rot.z;
        pose_msg.orientation.w = p.rot.w;
	
        // Joints
        sensor_msgs::JointState jointState_msg;
	
        // Get joints
        // To set Message form we need the FULL NAME
        jointState_msg.name.resize( mNumJoints );
        jointState_msg.position.resize( mNumJoints );
        for( int i = 0; i < mNumJoints; ++i ) {
            jointState_msg.name[i] = mFullJointNames[i];
            jointState_msg.position[i] = this->drchubo.mJoints[i]->GetAngle(0).Radian();
        }


        // Send them out
        posePub.publish( pose_msg );
        jointStatesPub.publish( jointState_msg );

        // **************************************************
        std::cout << "Releasing lock in UpdateStates" << std::endl;

    }

  
  
  /**
     * @function Robot::Load
     */
    void DRCPlugin::Robot::Load( physics::WorldPtr _world, 
                                 sdf::ElementPtr _sdf )
    {
        this->isInitialized = false;

        // load parameters
        if (_sdf->HasElement("drchubo") &&
            _sdf->GetElement("drchubo")->HasElement("model_name")) {
            this->model = _world->GetModel(_sdf->GetElement("drchubo")
                                           ->GetValueString("model_name"));
        }
        else {
            ROS_INFO("[DRCPLUGIN - Load] Can't find <drchubo><model_name> blocks. using default.");
            this->model = _world->GetModel("drchubo");
        }
  
        if (!this->model)
        {
            ROS_INFO("[DRCPLUGIN - Load] drchubo model not found.");
            return;
        }

        // Get hard-coded pin link
        this->pinLink = this->model->GetLink("Body_Torso");

        if( !this->pinLink )
        {
            ROS_ERROR("[DRCPLUGIN - Load] drchubo robot pin link not found.");
            return;
        }

        // Note: hardcoded link by name: @todo: make this a pugin param
        this->initialPose = this->pinLink->GetWorldPose();
        this->isInitialized = true;

        // Set ankleOffset
        this->ankleOffset = 0.20; //0.18;

        // Store joints
        this->mJoints.resize( mNumJoints );
        for( int i = 0; i < DRCPlugin::mNumJoints; ++i ) {
            if( this->model->GetJoint( DRCPlugin::mJointNames[i] ) ) {
                mJoints[i] = this->model->GetJoint( DRCPlugin::mJointNames[i] );
            }
            else {
                //printf("[DRCPlugin - RobotLoad] Joint %s did NOT load correctly for messaging \n", DRCPlugin::mJointNames[i].c_str() );
            }
        }


    }

////////////////////////////////////////////////////////////////////////////////
    void DRCPlugin::LoadDRCROSAPI()
    {

      std::string robot_grab_topic_name = "drc_world/robot_grab_link";
      ros::SubscribeOptions robot_grab_so =
	ros::SubscribeOptions::create<geometry_msgs::Pose>(
							   robot_grab_topic_name, 100,
							   boost::bind(&DRCPlugin::RobotGrabDrill, this, _1),
							   ros::VoidPtr(), &this->rosQueue);
      this->subRobotGrab = this->rosNode->subscribe(robot_grab_so);
      
      std::string robot_release_topic_name = "drc_world/robot_release_link";
      ros::SubscribeOptions robot_release_so =
	ros::SubscribeOptions::create<geometry_msgs::Pose>(
							   robot_release_topic_name, 100,
							   boost::bind(&DRCPlugin::RobotReleaseLink, this, _1),
							   ros::VoidPtr(), &this->rosQueue);
      this->subRobotRelease = this->rosNode->subscribe(robot_release_so);
      
    }

////////////////////////////////////////////////////////////////////////////////
    void DRCPlugin::LoadRobotROSAPI()
    {

        // Topic to set mode
        std::string pose_topic_name = "drchubo/set_pose";
        ros::SubscribeOptions pose_so =
            ros::SubscribeOptions::create<geometry_msgs::Pose>( pose_topic_name, 100,
                                                                boost::bind(&DRCPlugin::SetRobotPose, this, _1),
                                                                ros::VoidPtr(), &this->rosQueue);
        this->drchubo.subPose = this->rosNode->subscribe(pose_so);
  

        // Topic to set configuration
        std::string configuration_topic_name = "drchubo/configuration";
        ros::SubscribeOptions configuration_so =
            ros::SubscribeOptions::create<sensor_msgs::JointState>( configuration_topic_name, 100,
                                                                    boost::bind(&DRCPlugin::SetRobotConfiguration, this, _1),
                                                                    ros::VoidPtr(), &this->rosQueue );
        this->drchubo.subConfiguration = this->rosNode->subscribe(configuration_so);


        // Topic to set mode
        std::string mode_topic_name = "drchubo/mode";
        ros::SubscribeOptions mode_so =
            ros::SubscribeOptions::create<std_msgs::String>( mode_topic_name, 100,
                                                             boost::bind(&DRCPlugin::SetRobotModeTopic, this, _1),
                                                             ros::VoidPtr(), &this->rosQueue);
        this->drchubo.subMode = this->rosNode->subscribe(mode_so);  

        // Topic to set joint animation
        std::string jointAnimation_topic_name = "drchubo/jointAnimation";
        ros::SubscribeOptions jointAnimation_so =
            ros::SubscribeOptions::create<trajectory_msgs::JointTrajectory>( jointAnimation_topic_name, 100,
                                                                             boost::bind(&DRCPlugin::SetRobotJointAnimation, this, _1),
                                                                             ros::VoidPtr(), &this->rosQueue);
        this->drchubo.subJointAnimation = this->rosNode->subscribe(jointAnimation_so);  

        // Topic to set pose animation
        std::string poseAnimation_topic_name = "drchubo/poseAnimation";
        ros::SubscribeOptions poseAnimation_so =
            ros::SubscribeOptions::create<DRC_msgs::PoseStampedArray>( poseAnimation_topic_name, 100,
                                                                       boost::bind(&DRCPlugin::SetRobotPoseAnimation, this, _1),
                                                                       ros::VoidPtr(), &this->rosQueue);
        this->drchubo.subPoseAnimation = this->rosNode->subscribe(poseAnimation_so);  

        // Topic to set pose + joint animation
        std::string poseJointAnimation_topic_name = "drchubo/poseJointAnimation";
        ros::SubscribeOptions poseJointAnimation_so =
            ros::SubscribeOptions::create<DRC_msgs::PoseJointTrajectory>( poseJointAnimation_topic_name, 100,
                                                                          boost::bind(&DRCPlugin::SetRobotPoseJointAnimation, this, _1),
                                                                          ros::VoidPtr(), &this->rosQueue);
        this->drchubo.subPoseJointAnimation = this->rosNode->subscribe(poseJointAnimation_so);  

	// ************************
        //// Publishers
	// ************************
        // Publish joint state information
        jointStatesPub = this->rosNode->advertise<sensor_msgs::JointState>( "drchubo/jointStates",
									    100,
									    false );
        posePub = this->rosNode->advertise<geometry_msgs::Pose>( "drchubo/pose",
								 100, false );

    }


  //********************************************
  // SetRobotPose
  //********************************************
    void DRCPlugin::SetRobotPose(const geometry_msgs::Pose::ConstPtr &_pose) {

        math::Quaternion q(_pose->orientation.w, _pose->orientation.x,
                           _pose->orientation.y, _pose->orientation.z);
        q.Normalize();
        math::Pose pose(math::Vector3(_pose->position.x,
                                      _pose->position.y,
                                      _pose->position.z), q);
  
        this->drchubo.model->SetWorldPose(pose);

        // Store
        getCurrentPose();
    }


//******************************
// SetRobotConfiguration
//******************************
    void DRCPlugin::SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr &_cmd ) {
  
        std::map<std::string, double> joint_position_map;
  
        // Read the values and send them
        for (unsigned int i = 0; i < _cmd->name.size(); ++i)
        {  joint_position_map[ _cmd->name[i] ] = _cmd->position[i]; }

        // Send
        this->drchubo.model->SetJointPositions( joint_position_map ); 
  
        getCurrentJointState();

        ROS_INFO("[DRCPlugin] SET ROBOT CONFIGURATION SEE DEFAULTS \n");
        this->drchubo.modeType = ON_STAY_DOG_MODE;

        // Print the stay mode stuff
        //printf("[SET ROBOT CONFIGURATION] Robot is going to stay in position: %f %f %f rotation: %f, %f, %f, %f \n", 
               // defaultPose_p.pos.x,
               // defaultPose_p.pos.y, 
               // defaultPose_p.pos.z, 
               // defaultPose_p.rot.x, 
               // defaultPose_p.rot.y, 
               // defaultPose_p.rot.z, 
               // defaultPose_p.rot.w );

        //printf("[SET ROBOT CONFIGURATION] Joints: : \n" );
        for( std::map<std::string,double>::iterator iter=defaultJointState_p.begin();
             iter != defaultJointState_p.end(); ++iter ) {
            std::cout << (*iter).first << ": "<<(*iter).second << std::endl;
        }

    }

    //******************************
    // SetRobotPoseAnimation
    //******************************
    void DRCPlugin::SetRobotPoseAnimation(const DRC_msgs::PoseStampedArray::ConstPtr &_cmd ) {

        //printf("Setting Pose animation \n");
    
        // Set flag
        onPoseAnimation = true;
    
        // Fill
        int numTrajPoints = _cmd->poses.size();
        double T = _cmd->poses[numTrajPoints - 1].header.stamp.toSec();
        double t;
    
        gazebo::common::PoseAnimationPtr pose_anim( new gazebo::common::PoseAnimation( "test", T, false ) );
        gazebo::common::PoseKeyFrame *pose_key;
    
        for( int i = 0; i < numTrajPoints; ++i ) {
            t = _cmd->poses[i].header.stamp.toSec();    
            pose_key = pose_anim->CreateKeyFrame(t);
      
            pose_key->SetTranslation( math::Vector3( _cmd->poses[i].pose.position.x,
                                                     _cmd->poses[i].pose.position.y,
                                                     _cmd->poses[i].pose.position.z) );
      
            pose_key->SetRotation( math::Quaternion( _cmd->poses[i].pose.orientation.w,
                                                     _cmd->poses[i].pose.orientation.x,
                                                     _cmd->poses[i].pose.orientation.y,
                                                     _cmd->poses[i].pose.orientation.z ) );
        }
    
        // Attach the animation to the model
        this->drchubo.model->SetAnimation( pose_anim, poseAnim_callback );
        //printf("End loading Pose animation \n");
    
    }
  
    /**
     * @function SetRobotPoseJointAnimation
     */
    void DRCPlugin::SetRobotPoseJointAnimation(const DRC_msgs::PoseJointTrajectory::ConstPtr &_cmd ) {

        std::cout << "Aquiring lock in SetRobotPoseJointAnimation" << std::endl;
        boost::mutex::scoped_lock lock(this->update_mutex);


        this->world->EnablePhysicsEngine(false);
    
        //printf("Setting Robot pose + joint animation \n");

        // Set flag
        onJointAnimation = true;
        onPoseAnimation = true;

        // Set starting time

        // Fill joint initialization
        int numTrajPoints = _cmd->points.size();
        double T = _cmd->points[numTrajPoints - 1].time_from_start.toSec();
        double t;
    
        // Animation stuff
        std::map<std::string, common::NumericAnimationPtr> joint_anim;
        common::NumericKeyFrame *joint_key;
        gazebo::common::PoseAnimationPtr pose_anim( new gazebo::common::PoseAnimation( "test", T, false ) );
        gazebo::common::PoseKeyFrame *pose_key;
    
        // Store joint info
        for( unsigned int i = 0; i < _cmd->joint_names.size(); ++i ) {
            joint_anim[ _cmd->joint_names[i] ].reset( new common::NumericAnimation( "anim", T, false) );
        }
    
        // Save data
        for( int i = 0; i < numTrajPoints; ++i ) {
      
            t = _cmd->points[i].time_from_start.toSec();
            // Set all joints
            for( unsigned int j = 0; j < _cmd->points[i].positions.size(); ++j ) {
                joint_key = joint_anim[ _cmd->joint_names[j] ]->CreateKeyFrame(t);
                joint_key->SetValue( _cmd->points[i].positions[j] );
            }
      
            // Set poses
            pose_key = pose_anim->CreateKeyFrame(t);
      
            pose_key->SetTranslation( math::Vector3( _cmd->poses[i].position.x,
                                                     _cmd->poses[i].position.y,
                                                     _cmd->poses[i].position.z) );

            pose_key->SetRotation( math::Quaternion( _cmd->poses[i].orientation.w,
                                                     _cmd->poses[i].orientation.x,
                                                     _cmd->poses[i].orientation.y,
                                                     _cmd->poses[i].orientation.z ) );
      
        }
    
        // Attach the animation to the model
        this->drchubo.model->SetJointAnimation( joint_anim, jointAnim_callback );

        this->drchubo.model->SetAnimation( pose_anim, poseAnim_callback );

        
        this->world->EnablePhysicsEngine(true);
        

        //printf("End loading Joint + Pose animation \n");

        std::cout << "Releasing lock in SetRobotPoseJointAnimation" << std::endl;
    
    }



////////////////////////////////////////////////////////////////////////////////
    void DRCPlugin::SetRobotModeTopic(const std_msgs::String::ConstPtr &_str)
    {
        this->SetRobotMode(_str->data);
    }

////////////////////////////////////////////////////////////////////////////////
    void DRCPlugin::SetRobotMode(const std::string &_str)
    {

        // No gravity for all robot's joints
        if (_str == "no_gravity") {
            this->drchubo.model->SetGravityMode( false );
            this->drchubo.modeType = ON_NO_GRAVITY_MODE;
        }
        // Gravity only in left and right robot's feet
        else if( _str == "feet" ) {
            //printf("[INFO] Setting GRAVITY to true for feet \n");
            physics::Link_V links = this->drchubo.model->GetLinks();
            for( unsigned int i = 0; i < links.size(); ++i ) {
                // Probably not going to work with leftFoot and rightFoot since they are static
                if( links[i]->GetName() == "leftFoot" ) {
                    //printf("Setting left Foot WITH GRAVITY! \n");
                    links[i]->SetGravityMode( true );
                }
                else if( links[i]->GetName() == "rightFoot" ) {
                    //printf("Setting right Foot WITH GRAVITY! \n");
                    links[i]->SetGravityMode( true );
                }      
                // This is more likely to work
                else if( links[i]->GetName() == "Body_LAR" ) {
                    //printf("Setting Body_LAR WITH GRAVITY! \n");
                    links[i]->SetGravityMode( true );
                }      
                else if( links[i]->GetName() == "Body_RAR" ) {
                    //printf("Setting Body_RAR WITH GRAVITY! \n");
                    links[i]->SetGravityMode( true );
                }      

                else {
                    links[i]->SetGravityMode( false );
                }
            }
            this->drchubo.modeType = ON_FEET_MODE;
        }
        // Nominal gravity
        else if (_str == "nominal") {
            this->drchubo.model->SetGravityMode( true );
            this->drchubo.modeType = ON_NOMINAL_MODE;
        }

        // Stay at the current ModelState (pose + joints)
        else if (_str == "stay_dog") {
	  //ROS_INFO("[DRCPlugin] Set STAY_DOG mode \n");
            this->drchubo.modeType = ON_STAY_DOG_MODE;

            // Print the stay mode stuff
            //printf("[STAY-DOG] Robot is going to stay in position: %f %f %f rotation: %f, %f, %f, %f \n", 
                   // defaultPose_p.pos.x,
                   // defaultPose_p.pos.y, 
                   // defaultPose_p.pos.z, 
                   // defaultPose_p.rot.x, 
                   // defaultPose_p.rot.y, 
                   // defaultPose_p.rot.z, 
                   // defaultPose_p.rot.w );

            //printf("[STAY-DOG] Joints: : \n" );
	    /*
            for( std::map<std::string,double>::iterator iter=defaultJointState_p.begin();
                 iter != defaultJointState_p.end(); ++iter ) {
                std::cout << (*iter).first << ": "<<(*iter).second << std::endl;
		}*/

        }
  
        else {
            ROS_INFO("available modes:no_gravity, nominal (with gravity), feet (gravity only on both feet - SO FAR THIS GIVES STRANGE RESULTS. TRY USING NO_GRAVITY FOR ANIMATIONS)");
        }
    }

////////////////////////////////////////////////////////////////////////////////
    void DRCPlugin::SetRobotJointAnimation(const trajectory_msgs::JointTrajectory::ConstPtr &_cmd )
    {
        //printf("Setting Robot animation \n");
  
        // Set flag
        onJointAnimation = true;
  
        std::map<std::string, common::NumericAnimationPtr> joint_anim;
        common::NumericKeyFrame *joint_key;
  
        // Fill joint initialization
        int numTrajPoints = _cmd->points.size();
        double T = _cmd->points[numTrajPoints - 1].time_from_start.toSec();
        double t;
  
        for( unsigned int i = 0; i < _cmd->joint_names.size(); ++i ) {
            joint_anim[ _cmd->joint_names[i] ].reset( new common::NumericAnimation( "anim", T, false) );
        }
        
        for( int i = 0; i < numTrajPoints; ++i ) {
      
            t = _cmd->points[i].time_from_start.toSec();
            // Set all joints
            for( unsigned int j = 0; j < _cmd->points[i].positions.size(); ++j ) {
                joint_key = joint_anim[ _cmd->joint_names[j] ]->CreateKeyFrame(t);
                joint_key->SetValue( _cmd->points[i].positions[j] );
            }

        }
    
        // Attach the animation to the model
        this->drchubo.model->SetJointAnimation( joint_anim, jointAnim_callback );
        //printf("End loading Joint animation \n");

    }



/**
 * @function getCurrentJointState
 */
    void DRCPlugin::getCurrentJointState() { 

        defaultJointState_p.clear();
        // Get joints
        // To set Message form we need the FULL NAME
        for( int i = 0; i < mNumJoints; ++i ) {
            defaultJointState_p[mFullJointNames[i].c_str()] = this->drchubo.mJoints[i]->GetAngle(0).Radian();
        }
    }
    
/**
 * @function getCurrentPose
 */
    void DRCPlugin::getCurrentPose() {
  
        math::Pose pose = this->drchubo.model->GetWorldPose();
	math::Pose leftFootPose = this->drchubo.model->GetLink("Body_LAR")->GetWorldPose();
        defaultPose_p.pos.x = pose.pos.x;
        defaultPose_p.pos.y = pose.pos.y;
        defaultPose_p.pos.z = pose.pos.z; // - leftFootPose.pos.z) + this->drchubo.ankleOffset;

        defaultPose_p.rot.x = pose.rot.x;
        defaultPose_p.rot.y = pose.rot.y;
        defaultPose_p.rot.z = pose.rot.z;
        defaultPose_p.rot.w = pose.rot.w;
    }

    /**
     * @function jointAnimation_callback
     * @brief
     */
    void DRCPlugin::jointAnimation_callback() {
        //printf("[DRCPlugin - INFO] Joint Animation is OVER, set vel to zero \n");
        math::Vector3 linvel  = this->drchubo.model->GetWorldLinearVel();
        math::Vector3 angvel = this->drchubo.model->GetWorldAngularVel();

        //printf( "[DRCPlugin - INFO] Before zeroeing, the linear vel of drchubo was: %f %f %f \n", linvel.x, linvel.y, linvel.z );
        //printf( "[DRCPlugin - INFO] Before zeroeing, the angular vel: %f %f %f \n", angvel.x, angvel.y, angvel.z );
    
        this->drchubo.model->SetLinearVel( math::Vector3( 0, 0, 0 ) );
        this->drchubo.model->SetAngularVel( math::Vector3( 0, 0, 0 ) );

        this->drchubo.model->SetLinearAccel( math::Vector3( 0, 0, 0 ) );
        this->drchubo.model->SetAngularAccel( math::Vector3( 0, 0, 0 ) );

        // Set robot to stay dog mode
        this->SetRobotMode("stay_dog");
        // Set current configuration (last animation joint configuration) for the robot to stay
        getCurrentJointState();

        // Joint Animation is off
        this->onJointAnimation = false;
    }

//***************************************
// poseAnimation_callback
//*************************************** 
    void DRCPlugin::poseAnimation_callback() {
        //printf("Pose Animation is OVER \n");
        math::Vector3 linvel  = this->drchubo.model->GetWorldLinearVel();
        math::Vector3 angvel = this->drchubo.model->GetWorldAngularVel();
  
        //printf( "[DRCPlugin - INFO] Before zeroeing, the linear vel of drchubo was: %f %f %f \n", linvel.x, linvel.y, linvel.z );
        //printf( "[DRCPlugin - INFO] Before zeroeing, the angular vel: %f %f %f \n", angvel.x, angvel.y, angvel.z );    

        this->drchubo.model->SetLinearVel( math::Vector3( 0, 0, 0 ) );
        this->drchubo.model->SetAngularVel( math::Vector3( 0, 0, 0 ) );

        this->drchubo.model->SetLinearAccel( math::Vector3( 0, 0, 0 ) );
        this->drchubo.model->SetAngularAccel( math::Vector3( 0, 0, 0 ) );

        // Set robot to stay dog mode
        this->SetRobotMode("stay_dog");
        // Set current configuration (last animation joint configuration) for the robot to stay
        getCurrentPose();
  
        // Pose Animation is OFF
        this->onPoseAnimation = false;
  
    }


////////////////////////////////////////////////////////////////////////////////
    void DRCPlugin::ROSQueueThread()
    {
        static const double timeout = 0.01;

        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }


    /**
     * @function Robot::SetFootParallelToFloor
     */
    void DRCPlugin::Robot::SetFootParallelToFloor() {
        //printf("[DRCPlugin] Set foot parallel to floor \n");
        // Get World Pose
        math::Pose worldPose = this->model->GetWorldPose();
       
        math::Pose leftFootPose;
        // Get foot pose (let's assume left and right feet both have the same pose)
        if( !this->model->GetLink("leftFoot") ) {
            //printf(" [DRCPLUGIN - ParallelToFloor] Gazebo does not recognize fixed legFoot link as a proper link, let's try with Body_LAR...\n");
            if( this->model->GetLink("Body_LAR") ) {
                //printf("[DRCPLUGIN - ParallelToFloor] OK. LAR worked. This is upper than the real foot, so you know \n");
                leftFootPose = this->model->GetLink("Body_LAR")->GetWorldPose();
            }
            else {
                //printf( "[DRCPLUGIN - ParallelToFloor] None of them works, not setting foot parallel to floor \n");
                return;
            }
        } else { 
            //printf(" [DRCPLUGIN - ParallelToFloor] Using leftFoot as link of reference for setting foot parallel to floor"); 
            leftFootPose = this->model->GetLink("leftFoot")->GetWorldPose();
        } 
  
    
        // Set left foot parallel to plane, hopefully
        math::Pose invLeftFoot = leftFootPose.GetInverse();
        math::Pose worldPrime = worldPose; worldPrime.pos.z = 0;    
        math::Pose newPoseWorld = worldPrime*invLeftFoot*worldPose;
        newPoseWorld.pos.z = newPoseWorld.pos.z + this->ankleOffset; 

        this->model->SetWorldPose( newPoseWorld );
        printf( "[DRCPLUGIN - ParallelToFloor] Done  - setting foot up %f to account for ankle offset\n", this->ankleOffset );
        leftFootPose = this->model->GetLink("Body_LAR")->GetWorldPose();
        worldPose = this->model->GetWorldPose();
        printf( "Left foot pos: %f %f %f World pos of torso: %f %f %f \n", leftFootPose.pos.x, leftFootPose.pos.y, leftFootPose.pos.z, worldPose.pos.x, worldPose.pos.y, worldPose.pos.z);
    }  



////////////////////////////////////////////////////////////////////////////////
// dynamically add joint between 2 links
    physics::JointPtr DRCPlugin::AddJoint(physics::WorldPtr _world,
                                          physics::ModelPtr _model,
                                          physics::LinkPtr _link1,
                                          physics::LinkPtr _link2,
                                          std::string _type,
                                          math::Vector3 _anchor,
                                          math::Vector3 _axis,
                                          double _upper, double _lower,
                                          bool _disableCollision)
    {
        physics::JointPtr joint = _world->GetPhysicsEngine()->CreateJoint(
            _type, _model);
        joint->Attach(_link1, _link2);
        // load adds the joint to a vector of shared pointers kept
        // in parent and child links, preventing joint from being destroyed.
        joint->Load(_link1, _link2, math::Pose(_anchor, math::Quaternion()));
        // joint->SetAnchor(0, _anchor);
        joint->SetAxis(0, _axis);
        joint->SetHighStop(0, _upper);
        joint->SetLowStop(0, _lower);

        if (_link1)
            joint->SetName(_link1->GetName() + std::string("_") +
                           _link2->GetName() + std::string("_joint"));
        else
            joint->SetName(std::string("world_") +
                           _link2->GetName() + std::string("_joint"));
        joint->Init();


        // disable collision between the link pair
        if (_disableCollision)
        {
            if (_link1)
                _link1->SetCollideMode("fixed");
            if (_link2)
                _link2->SetCollideMode("fixed");
        }


        return joint;
    }



////////////////////////////////////////////////////////////////////////////////
// remove a joint
    void DRCPlugin::RemoveJoint(physics::JointPtr &_joint)
    {
        bool paused = this->world->IsPaused();
        this->world->SetPaused(true);
        if (_joint)
        {
            // reenable collision between the link pair
            physics::LinkPtr parent = _joint->GetParent();
            physics::LinkPtr child = _joint->GetChild();
            if (parent)
                parent->SetCollideMode("all");
            if (child)
                child->SetCollideMode("all");

            _joint->Detach();
            _joint.reset();
        }
        this->world->SetPaused(paused);
    }

////////////////////////////////////////////////////////////////////////////////
    void DRCPlugin::Teleport( const physics::LinkPtr &_pinLink,
                              physics::JointPtr &_pinJoint,
                              const math::Pose &_pose,
                              const std::map<std::string, double> &/*_jp*/)
    {
        this->Teleport(_pinLink, _pinJoint, _pose);
        /// \todo: use _jp to set robot configuration
    }

////////////////////////////////////////////////////////////////////////////////
    void DRCPlugin::Teleport( const physics::LinkPtr &_pinLink,
                              physics::JointPtr &_pinJoint,
                              const math::Pose &_pose )
    {
        // pause, break joint, update pose, create new joint, unpause
        bool p = this->world->IsPaused();
        bool e = this->world->GetEnablePhysicsEngine();
        this->world->EnablePhysicsEngine(false);
        this->world->SetPaused(true);
        if (_pinJoint)
            this->RemoveJoint(_pinJoint);
        _pinLink->GetModel()->SetLinkWorldPose(_pose, _pinLink);
        if (!_pinJoint)
            _pinJoint = this->AddJoint(this->world,
                                       _pinLink->GetModel(),
                                       physics::LinkPtr(),
                                       this->drchubo.pinLink,
                                       "revolute",
                                       math::Vector3(0, 0, 0),
                                       math::Vector3(0, 0, 1),
                                       0.0, 0.0);
        this->world->SetPaused(p);
        this->world->EnablePhysicsEngine(e);
    }
 
 
} // gazebo namespace
