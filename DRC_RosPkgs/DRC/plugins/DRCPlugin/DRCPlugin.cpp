/*
 * @file DRCPlugin.cpp
 * @brief
 */

#include <map>
#include <string>
#include <stdlib.h>

#include "DRCPlugin.h"

// Gazebo Animation 
#include <common/common.hh>

namespace gazebo
{
  GZ_REGISTER_WORLD_PLUGIN(DRCPlugin)

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


  // Setup ROS interfaces for robot
  this->LoadRobotROSAPI();
  
  // Set robot mode to no_gravity to see what happens
  this->SetRobotMode( "no_gravity" );
  printf("Set ROBOT MODE To NO GRAVITY ! \n");

  // ros callback queue for processing subscription
  this->callbackQueueThread = boost::thread(
    boost::bind(&DRCPlugin::ROSQueueThread, this));

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&DRCPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void DRCPlugin::SetRobotPose(const geometry_msgs::Pose::ConstPtr &_pose)
{
  math::Quaternion q(_pose->orientation.w, _pose->orientation.x,
                     _pose->orientation.y, _pose->orientation.z);
  q.Normalize();
  math::Pose pose(math::Vector3(_pose->position.x,
                                _pose->position.y,
                                _pose->position.z), q);
  this->drchubo.model->SetWorldPose(pose);
}


////////////////////////////////////////////////////////////////////////////////
void DRCPlugin::RobotReleaseLink(const geometry_msgs::Pose::ConstPtr &/*_cmd*/)
{
  //  this->RemoveJoint(this->grabJoint);
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

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void DRCPlugin::UpdateStates()
{
  double curTime = this->world->GetSimTime().Double();


  if (curTime > this->lastUpdateTime)
  {
    double dt = curTime - this->lastUpdateTime;
  }
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


////////////////////////////////////////////////////////////////////////////////
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
    ROS_INFO("Can't find <drchubo><model_name> blocks. using default.");
    this->model = _world->GetModel("drchubo");
  }
  
  if (!this->model)
    {
    ROS_INFO("drchubo model not found.");
    return;
  }

  // Get hard-coded pin link
  this->pinLink = this->model->GetLink("Body_Torso");

  if( !this->pinLink )
  {
    ROS_ERROR("drchubo robot pin link not found.");
    return;
  }

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->initialPose = this->pinLink->GetWorldPose();
  this->isInitialized = true;
}

////////////////////////////////////////////////////////////////////////////////
void DRCPlugin::LoadDRCROSAPI()
{
  /*
    std::string robot_exit_car_topic_name = "drc_world/robot_exit_car";
    ros::SubscribeOptions robot_exit_car_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      robot_exit_car_topic_name, 100,
      boost::bind(&VRCPlugin::RobotExitCar, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->subRobotExitCar = this->rosNode->subscribe(robot_exit_car_so);
  */
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
}

////////////////////////////////////////////////////////////////////////////////
void DRCPlugin::SetRobotJointAnimation(const trajectory_msgs::JointTrajectory::ConstPtr &_cmd )
{
  printf("Setting Robot animation \n");
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
  this->drchubo.model->SetJointAnimation( joint_anim );
  printf("End loading Joint animation \n");

}

  ////////////////////////////////////////////////////////////////////////////////
  void DRCPlugin::SetRobotPoseAnimation(const DRC_msgs::PoseStampedArray::ConstPtr &_cmd )
  {
  printf("Setting Pose animation \n");

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
    pose_key->SetRotation( math::Quaternion( 0.0, 0.0, 0.0 ) );
/*    pose_key->SetRotation( math::Quaternion( _cmd->poses[i].pose.orientation.x,
					     _cmd->poses[i].pose.orientation.y,
					     _cmd->poses[i].pose.orientation.z,
					     _cmd->poses[i].pose.orientation.w ) );*/
  }
    
  // Attach the animation to the model
  this->drchubo.model->SetAnimation( pose_anim );
  printf("End loading Pose animation \n");

}

  /**
   * @function SetRobotPoseJointAnimation
   */
  void DRCPlugin::SetRobotPoseJointAnimation(const DRC_msgs::PoseJointTrajectory::ConstPtr &_cmd ) {
    
    printf("Setting Robot pose + joint animation \n");
    
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
      pose_key->SetRotation( math::Quaternion( 0.0, 0.0, 0.0 ) );
      /*    pose_key->SetRotation( math::Quaternion( _cmd->poses[i].pose.orientation.x,
	    _cmd->poses[i].pose.orientation.y,
	    _cmd->poses[i].pose.orientation.z,
	    _cmd->poses[i].pose.orientation.w ) );*/
      
    }
    
    // Attach the animation to the model
    this->drchubo.model->SetJointAnimation( joint_anim );
    this->drchubo.model->SetAnimation( pose_anim );

    printf("End loading Joint + Pose animation \n");
    
  }


////////////////////////////////////////////////////////////////////////////////
  void DRCPlugin::SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr &_cmd )
{
  std::map<std::string, double> joint_position_map;
  
  // Read the values and send them
  for (unsigned int i = 0; i < _cmd->name.size(); ++i)
  {  joint_position_map[ _cmd->name[i] ] = _cmd->position[i]; }

  // Send
  this->drchubo.model->SetJointPositions( joint_position_map ); 
      
}

////////////////////////////////////////////////////////////////////////////////
void DRCPlugin::SetRobotModeTopic(const std_msgs::String::ConstPtr &_str)
{
  this->SetRobotMode(_str->data);
}

////////////////////////////////////////////////////////////////////////////////
void DRCPlugin::SetRobotMode(const std::string &_str)
{
  if (_str == "no_gravity") {
    this->drchubo.model->SetGravityMode( false );
  }
  else if (_str == "nominal") {
    this->drchubo.model->SetGravityMode( true );
  }

  else {
    ROS_INFO("available modes:no_gravity, nominal (with gravity)");
  }
}



} // gazebo namespace
