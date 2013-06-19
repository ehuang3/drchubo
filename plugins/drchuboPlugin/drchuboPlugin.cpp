/*
 * @file drchuboPlugin.cpp
 * @author A. Huaman
 */

#include <map>
#include <string>
#include <stdlib.h>

#include "VRCPlugin.h"

namespace gazebo {
  
  GZ_REGISTER_WORLD_PLUGIN( drchuboPlugin )
  
  /**
   * @function Constructor
   */
  drchuboPlugin::drchuboPlugin() {
    /// initial anchor pose
    this->warpRobotWithCmdVel = false;
  }

  /**
   * @function Destructor
   */
  drchuboPlugin::~drchuboPlugin() {

    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    this->rosNode->shutdown();
    this->rosQueue.clear();
    this->rosQueue.disable();
    this->callbackQueueThread.join();
    delete this->rosNode;
  }
  
  /**
   * @function Load
   * @brief Load the controller
   */
  void drchuboPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

    // save pointers
    this->world = _parent;
    this->sdf = _sdf;
    

    // ros callback queue for processing subscription
    // this->deferredLoadThread = boost::thread(
    //   boost::bind(&drchuboPlugin::DeferredLoad, this));
    this->DeferredLoad();
  }
  
  /**
   * @function
   * @brief Load the controller
   */
  void drchuboPlugin::DeferredLoad() {

    // initialize ros
    if (!ros::isInitialized()) {
      gzerr << "Not loading drchubo plugin since ROS hasn't been "
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
    this->robotCmdVel = geometry_msgs::Twist();
    
    // Load Robot
    this->drchubo.Load(this->world, this->sdf);
    
    // Setup ROS interfaces for robot
    this->LoadRobotROSAPI();

  // Harness the Robot
  // On startup, simulate "virtual harness" by turning gravity off
  // allowing the controllers can initialize without the robot falling
  if (this->atlas.isInitialized)
  {
    if {
      this->SetRobotMode("pinned");
      this->drchubo.startupHarness = true;
      ROS_DEBUG("Start robot with gravity turned off and harnessed.");
      if (math::equal(this->drchubo.startupHarnessDuration, 0.0))
        ROS_DEBUG("drchubo will stay pinned.");
      else
        ROS_DEBUG("Resume to nominal mode after %f seconds.",
		  this->drchubo.startupHarnessDuration);
    }
  }
  
  // ros callback queue for processing subscription
  this->callbackQueueThread = boost::thread(
					    boost::bind(&drchuboPlugin::ROSQueueThread, this));

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
								  boost::bind(&drchuboPlugin::UpdateStates, this));
  }
  
  /**
   * @function SetRobotModeTopic
   */
  void drchuboPlugin::SetRobotModeTopic(const std_msgs::String::ConstPtr &_str)
{
  this->SetRobotMode( _str->data );
}

  /**
   * @function SetRobotMode
   */
  void drchuboPlugin::SetRobotMode(const std::string &_str) {
    if (_str == "no_gravity")
      {
	// stop warping robot
	this->warpRobotWithCmdVel = false;
	physics::Link_V links = this->atlas.model->GetLinks();
	for (unsigned int i = 0; i < links.size(); ++i)
	  {
	    links[i]->SetGravityMode(false);
	  }
	if (this->atlas.pinJoint)
	  this->RemoveJoint(this->atlas.pinJoint);
	if (this->vehicleRobotJoint)
	  this->RemoveJoint(this->vehicleRobotJoint);
      }
    else if (_str == "feet")
      {
	// stop warping robot
	this->warpRobotWithCmdVel = false;
	physics::Link_V links = this->atlas.model->GetLinks();
	for (unsigned int i = 0; i < links.size(); ++i)
	  {
	    if (links[i]->GetName() == "l_foot" || links[i]->GetName() == "r_foot")
	      links[i]->SetGravityMode(true);
	    else
	      links[i]->SetGravityMode(false);
	  }
	if (this->atlas.pinJoint)
	  this->RemoveJoint(this->atlas.pinJoint);
	if (this->vehicleRobotJoint)
	  this->RemoveJoint(this->vehicleRobotJoint);
      }
    else if (_str == "harnessed")
      {
	bool paused = this->world->IsPaused();
	this->world->SetPaused(true);
	
    // remove pin
	if (this->drchubo.pinJoint) {
	  this->RemoveJoint(this->drchubo.pinJoint);
	}

    // raise robot, find ground height, set it down and upright it, then pin it
    math::Pose atlasPose = this->atlas.pinLink->GetWorldPose();

    // where to raise robot to
    math::Pose atlasAway = atlasPose + math::Pose(0, 0, 50.0, 0, 0, 0);

    // move robot out of the way
    this->atlas.model->SetLinkWorldPose(atlasAway, this->atlas.pinLink);

    // where to start down casting ray to check for ground
    math::Pose rayStart = atlasPose - math::Pose(0, 0, -2.0, 0, 0, 0);

    physics::EntityPtr objectBelow =
      this->world->GetEntityBelowPoint(rayStart.pos);
    if (objectBelow)
    {
      math::Box groundBB = objectBelow->GetBoundingBox();
      double groundHeight = groundBB.max.z;

      // gzdbg << objectBelow->GetName() << "\n";
      // gzdbg << objectBelow->GetParentModel()->GetName() << "\n";
      // gzdbg << groundHeight << "\n";
      // gzdbg << groundBB.max.z << "\n";
      // gzdbg << groundBB.min.z << "\n";

      // slightly above ground and upright
      atlasPose.pos.z = groundHeight + 1.11;
      atlasPose.rot.SetFromEuler(0, 0, 0);
      this->atlas.model->SetLinkWorldPose(atlasPose, this->atlas.pinLink);

      this->atlas.pinJoint = this->AddJoint(this->world,
                                        this->atlas.model,
                                        physics::LinkPtr(),
                                        this->atlas.pinLink,
                                        "revolute",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        0.0, 0.0);
      this->atlas.initialPose = this->atlas.pinLink->GetWorldPose();

      // turning off effect of gravity
      physics::Link_V links = this->atlas.model->GetLinks();
      for (unsigned int i = 0; i < links.size(); ++i)
      {
        links[i]->SetGravityMode(false);
      }
    }
    else
    {
      gzwarn << "No entity below robot, or GetEntityBelowPoint "
             << "returned NULL pointer.\n";
      // put atlas back
      this->atlas.model->SetLinkWorldPose(atlasPose, this->atlas.pinLink);
    }
    this->world->SetPaused(paused);
  }
  else if (_str == "pinned")
  {
    // pinning robot, and turning off effect of gravity
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
    if (!this->atlas.pinJoint)
      this->atlas.pinJoint = this->AddJoint(this->world,
                                        this->atlas.model,
                                        physics::LinkPtr(),
                                        this->atlas.pinLink,
                                        "revolute",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        0.0, 0.0);
    this->atlas.initialPose = this->atlas.pinLink->GetWorldPose();

    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
  }
  else if (_str == "pinned_with_gravity")
  {
    // pinning robot, and turning off effect of gravity
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
    if (!this->atlas.pinJoint)
      this->atlas.pinJoint = this->AddJoint(this->world,
                                        this->atlas.model,
                                        physics::LinkPtr(),
                                        this->atlas.pinLink,
                                        "revolute",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        0.0, 0.0);
    this->atlas.initialPose = this->atlas.pinLink->GetWorldPose();

    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
  }
  else if (_str == "nominal")
  {
    // nominal
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
  }
  else if (_str == "bdi_stand")
  {
    // reset some flags
    this->atlas.startupBDIStand = false;
    this->atlas.bdiStandNominal = false;

    // pin robot
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
    if (this->vehicleRobotJoint)
      this->RemoveJoint(this->vehicleRobotJoint);
    this->atlas.pinJoint = this->AddJoint(this->world,
                                      this->atlas.model,
                                      physics::LinkPtr(),
                                      this->atlas.pinLink,
                                      "revolute",
                                      math::Vector3(0, 0, 0),
                                      math::Vector3(0, 0, 1),
                                      0.0, 0.0);
    // turning off effect of gravity
    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }

    // turn physics off while manipulating things
    bool physics = this->world->GetEnablePhysicsEngine();
    bool paused = this->world->IsPaused();
    this->world->SetPaused(true);
    this->world->EnablePhysicsEngine(false);

    // set robot configuration
    this->atlasCommandController.SetPIDStand(this->atlas.model);
    /// FIXME: uncomment sleep below and AtlasSimInterface fails to STAND, why?
    // gazebo::common::Time::Sleep(gazebo::common::Time(1.0));
    ROS_INFO("set robot configuration done");

    this->world->EnablePhysicsEngine(physics);
    this->world->SetPaused(paused);
    // this->atlasCommandController.SetBDIFREEZE();

    // start the rest of the sequence
    this->atlas.startupBDIStand = true;
    this->atlas.startupBDIStandStartTime = this->world->GetSimTime();
  }
  else
  {
    ROS_INFO("available modes:no_gravity, feet, pinned, nominal");
  }
}

////////////////////////////////////////////////////////////////////////////////
void drchuboPlugin::SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd)
{
  if (_cmd->linear.x == 0 && _cmd->linear.y == 0 && _cmd->angular.z == 0)
  {
    this->warpRobotWithCmdVel = false;
  }
  else
  {
    this->robotCmdVel = *_cmd;
    this->warpRobotWithCmdVel = true;
    this->lastUpdateTime = this->world->GetSimTime().Double();
  }
}

  /**
   * @function SetRobotPose
   */
  void drchuboPlugin::SetRobotPose(const geometry_msgs::Pose::ConstPtr &_pose) {

    math::Quaternion q( _pose->orientation.w, _pose->orientation.x,
			_pose->orientation.y, _pose->orientation.z );
    q.Normalize();
    math::Pose pose( math::Vector3( _pose->position.x,
				    _pose->position.y,
				    _pose->position.z ), q );
    this->drchubo.model->SetWorldPose(pose);
}

  /**
   * @function Teleport
   */
  void drchuboPlugin::Teleport( const physics::LinkPtr &_pinLink,
				physics::JointPtr &_pinJoint,
				const math::Pose &_pose,
				const std::map<std::string, double> &/*_jp*/ ) {

    this->Teleport( _pinLink, 
		    _pinJoint, 
		    _pose );
  /// \todo: use _jp to set robot configuration
}

  /**
   * @function Teleport
   */
  void drchuboPlugin::Teleport( const physics::LinkPtr &_pinLink,
				physics::JointPtr &_pinJoint,
				const math::Pose &_pose ) {
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

  /**
   * @function UpdateStates
   * @brief Play the trajectory, update states
   */
  void drchuboPlugin::UpdateStates() {

    double curTime = this->world->GetSimTime().Double();

    // if user chooses bdi_stand mode, robot will be initialized
    // with PID stand in BDI stand pose pinned.
    // After startupStandPrepDuration - 1 seconds, pin released.
    // After startupStandPrepDuration seconds, start Stand mode
    if ( this->atlas.startupBDIStand && this->atlas.isInitialized ) {
      if ((curTime - this->atlas.startupBDIStandStartTime.Double()) >
	  atlas.startupStandPrepDuration)
	{
	  ROS_DEBUG("going into Stand");
	  this->atlasCommandController.SetBDIStand();
      this->atlas.startupBDIStand = false;
    }
    else if (!this->atlas.bdiStandNominal &&
      (curTime - this->atlas.startupBDIStandStartTime.Double()) >
      (atlas.startupStandPrepDuration - 1.0))
    {
      ROS_DEBUG("going into Nominal");
      this->SetRobotMode("nominal");
      this->atlas.bdiStandNominal = true;
    }
  }

  if (this->atlas.startupHarness && this->atlas.isInitialized &&
      !math::equal(atlas.startupHarnessDuration, 0.0) &&
      curTime > atlas.startupHarnessDuration)
  {
    this->SetRobotMode("nominal");
    this->atlas.startupHarness = false;
  }

  if (curTime > this->lastUpdateTime)
  {
    this->CheckThreadStart();

    double dt = curTime - this->lastUpdateTime;

    if (this->warpRobotWithCmdVel)
    {
      this->lastUpdateTime = curTime;
      math::Pose cur_pose = this->atlas.pinLink->GetWorldPose();
      math::Pose new_pose = cur_pose;

      // increment x,y in cur_pose frame
      math::Vector3 cmd(this->robotCmdVel.linear.x,
                        this->robotCmdVel.linear.y, 0);
      cmd = cur_pose.rot.RotateVector(cmd);

      new_pose.pos = cur_pose.pos + cmd * dt;
      // prevent robot from drifting vertically
      new_pose.pos.z = this->atlas.initialPose.pos.z;

      math::Vector3 rpy = cur_pose.rot.GetAsEuler();
      // decay non-yaw tilts
      rpy.x = 0;
      rpy.y = 0;
      rpy.z = rpy.z + this->robotCmdVel.angular.z * dt;

      new_pose.rot.SetFromEuler(rpy);

      // set this as the new anchor pose of the pin joint
      this->Teleport(this->atlas.pinLink,
                     this->atlas.pinJoint,
                     new_pose);
    }
  }
  }
  
  /**
   * @function ROSQueueThread
   */
void drchuboPlugin::ROSQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

  /**
   * @function CheckThreadStart
   */
  void drchuboPlugin::CheckThreadStart() {
  }    


////////////////////////////////////////////////////////////////////////////////
void drchuboPlugin::Robot::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->isInitialized = false;
  this->startupHarnessDuration = 10;
  this->startupStandPrepDuration = 2;
  this->startupHarness = false;
  this->startupBDIStand = false;
  this->startupMode = "bdi_stand";

  // load parameters
  if (_sdf->HasElement("atlas") &&
      _sdf->GetElement("atlas")->HasElement("model_name"))
  {
    this->model = _world->GetModel(_sdf->GetElement("atlas")
                        ->GetValueString("model_name"));
  }
  else
  {
    ROS_INFO("Can't find <atlas><model_name> blocks. using default.");
    this->model = _world->GetModel("atlas");
  }

  if (!this->model)
  {
    ROS_INFO("atlas model not found.");
    return;
  }

  if (_sdf->HasElement("atlas") &&
      _sdf->GetElement("atlas")->HasElement("pin_link"))
  {
    this->pinLink = this->model->GetLink(_sdf->GetElement("atlas")
                        ->GetValueString("pin_link"));
  }
  else
  {
    ROS_INFO("Can't find <atlas><pin_link> blocks, using default.");
    this->pinLink = this->model->GetLink("utorso");
  }

  if (!this->pinLink)
  {
    ROS_ERROR("atlas robot pin link not found.");
    return;
  }

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->initialPose = this->pinLink->GetWorldPose();
  this->isInitialized = true;
}

////////////////////////////////////////////////////////////////////////////////
void drchuboPlugin::LoadVRCROSAPI()
{
  if (this->cheatsEnabled)
  {
    // ros subscription
    std::string robot_enter_car_topic_name = "drc_world/robot_enter_car";
    ros::SubscribeOptions robot_enter_car_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      robot_enter_car_topic_name, 100,
      boost::bind(&drchuboPlugin::RobotEnterCar, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->subRobotEnterCar = this->rosNode->subscribe(robot_enter_car_so);

    std::string robot_exit_car_topic_name = "drc_world/robot_exit_car";
    ros::SubscribeOptions robot_exit_car_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      robot_exit_car_topic_name, 100,
      boost::bind(&drchuboPlugin::RobotExitCar, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->subRobotExitCar = this->rosNode->subscribe(robot_exit_car_so);

    std::string robot_grab_topic_name = "drc_world/robot_grab_link";
    ros::SubscribeOptions robot_grab_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      robot_grab_topic_name, 100,
      boost::bind(&drchuboPlugin::RobotGrabFireHose, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->subRobotGrab = this->rosNode->subscribe(robot_grab_so);

    std::string robot_release_topic_name = "drc_world/robot_release_link";
    ros::SubscribeOptions robot_release_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      robot_release_topic_name, 100,
      boost::bind(&drchuboPlugin::RobotReleaseLink, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->subRobotRelease = this->rosNode->subscribe(robot_release_so);
  }
}

////////////////////////////////////////////////////////////////////////////////
void drchuboPlugin::LoadRobotROSAPI()
{
  if (!this->rosNode->getParam("atlas/time_to_unpin",
    atlas.startupHarnessDuration))
  {
    ROS_DEBUG("atlas/time_to_unpin not specified, default harness duration to"
             " %f seconds", atlas.startupHarnessDuration);
  }

  if (!this->rosNode->getParam("atlas/startup_mode", atlas.startupMode))
  {
    ROS_INFO("atlas/startup_mode not specified, default bdi_stand that "
             " takes %f seconds to finish.", atlas.startupStandPrepDuration);
  }
  else if (atlas.startupMode == "bdi_stand")
  {
    ROS_INFO("Starting robot with BDI standing");
  }
  else if (atlas.startupMode == "pinned")
  {
    ROS_INFO("Starting robot pinned");
  }
  else
  {
    ROS_ERROR("Unsupported /atlas/startup_mode [%s]",
      atlas.startupMode.c_str());
  }

  if (this->cheatsEnabled)
  {
    // ros subscription
    std::string trajectory_topic_name = "atlas/cmd_vel";
    ros::SubscribeOptions trajectory_so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(
      trajectory_topic_name, 100,
      boost::bind(&drchuboPlugin::SetRobotCmdVel, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->atlas.subTrajectory = this->rosNode->subscribe(trajectory_so);

    std::string pose_topic_name = "atlas/set_pose";
    ros::SubscribeOptions pose_so =
      ros::SubscribeOptions::create<geometry_msgs::Pose>(
      pose_topic_name, 100,
      boost::bind(&drchuboPlugin::SetRobotPose, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->atlas.subPose = this->rosNode->subscribe(pose_so);

    std::string configuration_topic_name = "atlas/configuration";
    ros::SubscribeOptions configuration_so =
      ros::SubscribeOptions::create<sensor_msgs::JointState>(
      configuration_topic_name, 100,
      boost::bind(&drchuboPlugin::SetRobotConfiguration, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->atlas.subConfiguration =
      this->rosNode->subscribe(configuration_so);

    std::string mode_topic_name = "atlas/mode";
    ros::SubscribeOptions mode_so =
      ros::SubscribeOptions::create<std_msgs::String>(
      mode_topic_name, 100,
      boost::bind(&drchuboPlugin::SetRobotModeTopic, this, _1),
      ros::VoidPtr(), &this->rosQueue);
    this->atlas.subMode = this->rosNode->subscribe(mode_so);
  }
}

////////////////////////////////////////////////////////////////////////////////
void drchuboPlugin::SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr
  &_cmd)
{
  // This function is planned but not yet implemented.
  ROS_ERROR("The atlas/configuration handler is not implemented.\n");
/*
  for (unsigned int i = 0; i < _cmd->name.size(); ++i)
  {
    this->atlas.model->SetJointPositions();
  }
*/
}

////////////////////////////////////////////////////////////////////////////////
drchuboPlugin::AtlasCommandController::AtlasCommandController()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading AtlasCommandController since ROS hasn't been "
          << "properly initialized.  Try starting Gazebo with"
          << " ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // must match those inside AtlasPlugin
  this->jointNames.push_back("atlas::back_lbz");
  this->jointNames.push_back("atlas::back_mby");
  this->jointNames.push_back("atlas::back_ubx");
  this->jointNames.push_back("atlas::neck_ay");
  this->jointNames.push_back("atlas::l_leg_uhz");
  this->jointNames.push_back("atlas::l_leg_mhx");
  this->jointNames.push_back("atlas::l_leg_lhy");
  this->jointNames.push_back("atlas::l_leg_kny");
  this->jointNames.push_back("atlas::l_leg_uay");
  this->jointNames.push_back("atlas::l_leg_lax");
  this->jointNames.push_back("atlas::r_leg_uhz");
  this->jointNames.push_back("atlas::r_leg_mhx");
  this->jointNames.push_back("atlas::r_leg_lhy");
  this->jointNames.push_back("atlas::r_leg_kny");
  this->jointNames.push_back("atlas::r_leg_uay");
  this->jointNames.push_back("atlas::r_leg_lax");
  this->jointNames.push_back("atlas::l_arm_usy");
  this->jointNames.push_back("atlas::l_arm_shx");
  this->jointNames.push_back("atlas::l_arm_ely");
  this->jointNames.push_back("atlas::l_arm_elx");
  this->jointNames.push_back("atlas::l_arm_uwy");
  this->jointNames.push_back("atlas::l_arm_mwx");
  this->jointNames.push_back("atlas::r_arm_usy");
  this->jointNames.push_back("atlas::r_arm_shx");
  this->jointNames.push_back("atlas::r_arm_ely");
  this->jointNames.push_back("atlas::r_arm_elx");
  this->jointNames.push_back("atlas::r_arm_uwy");
  this->jointNames.push_back("atlas::r_arm_mwx");

  unsigned int n = this->jointNames.size();
  this->ac.position.resize(n);
  this->ac.velocity.resize(n);
  this->ac.effort.resize(n);
  this->ac.kp_position.resize(n);
  this->ac.ki_position.resize(n);
  this->ac.kd_position.resize(n);
  this->ac.kp_velocity.resize(n);
  this->ac.i_effort_min.resize(n);
  this->ac.i_effort_max.resize(n);
  this->ac.k_effort.resize(n);

  for (unsigned int i = 0; i < n; ++i)
  {
    std::vector<std::string> pieces;
    boost::split(pieces, this->jointNames[i], boost::is_any_of(":"));

    double val;
    this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
      "/p", val);
    this->ac.kp_position[i] = val;

    this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
      "/i", val);
    this->ac.ki_position[i] = val;

    this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
      "/d", val);
    this->ac.kd_position[i] = val;

    this->rosNode->getParam("atlas_controller/gains/" + pieces[2] +
      "/i_clamp", val);
    this->ac.i_effort_min[i] = -val;
    this->ac.i_effort_max[i] = val;
    this->ac.k_effort[i] =  255;

    this->ac.velocity[i]     = 0;
    this->ac.effort[i]       = 0;
    this->ac.kp_velocity[i]  = 0;
  }

  this->pubAtlasCommand =
    this->rosNode->advertise<atlas_msgs::AtlasCommand>(
    "/atlas/atlas_command", 1, true);

  this->pubAtlasSimInterfaceCommand =
    this->rosNode->advertise<atlas_msgs::AtlasSimInterfaceCommand>(
    "/atlas/atlas_sim_interface_command", 1, true);

  // ros::SubscribeOptions jointStatesSo =
  //   ros::SubscribeOptions::create<sensor_msgs::JointState>(
  //   "/atlas/joint_states", 1,
  //   boost::bind(&AtlasCommandController::GetJointStates, this, _1),
  //   ros::VoidPtr(), this->rosNode->getCallbackQueue());
  // this->subJointStates =
  //   this->rosNode->subscribe(jointStatesSo);
}

////////////////////////////////////////////////////////////////////////////////
drchuboPlugin::AtlasCommandController::~AtlasCommandController()
{
  this->rosNode->shutdown();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
void drchuboPlugin::AtlasCommandController::GetJointStates(
        const sensor_msgs::JointState::ConstPtr &_js)
{
  /// \todo: implement joint state monitoring when setting configuration
}

////////////////////////////////////////////////////////////////////////////////
void drchuboPlugin::AtlasCommandController::SetPIDStand(
  physics::ModelPtr atlasModel)
{
  // seated configuration
  this->ac.header.stamp = ros::Time::now();


  // StandPrep end pose --> Stand  pose
  this->ac.position[0]  =   2.438504816382192e-05;
  this->ac.position[1]  =   0.0015186156379058957;
  this->ac.position[2]  =   9.983908967114985e-06;
  this->ac.position[3]  =   -0.0010675729718059301;
  this->ac.position[4]  =   -0.0003740221436601132;
  this->ac.position[5]  =   0.06201673671603203;
  this->ac.position[6]  =  -0.2333149015903473;
  this->ac.position[7]  =   0.5181407332420349;
  this->ac.position[8]  =  -0.27610817551612854;
  this->ac.position[9]  =   -0.062101610004901886;
  this->ac.position[10] =  0.00035181696875952184;
  this->ac.position[11] =   -0.06218484416604042;
  this->ac.position[12] =  -0.2332201600074768;
  this->ac.position[13] =   0.51811283826828;
  this->ac.position[14] =  -0.2762000858783722;
  this->ac.position[15] =   0.06211360543966293;
  this->ac.position[16] =   0.29983898997306824;
  this->ac.position[17] =   -1.303462266921997;
  this->ac.position[18] =   2.0007927417755127;
  this->ac.position[19] =   0.49823325872421265;
  this->ac.position[20] =  0.0003098883025813848;
  this->ac.position[21] =   -0.0044272784143686295;
  this->ac.position[22] =   0.29982614517211914;
  this->ac.position[23] =   1.3034454584121704;
  this->ac.position[24] =   2.000779867172241;
  this->ac.position[25] =  -0.498238742351532;
  this->ac.position[26] =  0.0003156556049361825;
  this->ac.position[27] =   0.004448802210390568;


  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    this->ac.k_effort[i] =  255;

  // set joint positions
  std::map<std::string, double> jps;
  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
    jps.insert(std::make_pair(this->jointNames[i], this->ac.position[i]));

  atlasModel->SetJointPositions(jps);

  // publish AtlasCommand
  this->pubAtlasCommand.publish(ac);
}


}
