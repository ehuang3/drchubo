/**
 * @file DRCPlugin.h
 */

#ifndef GAZEBO_CDR_PLUGIN_HH
#define GAZEBO_CDR_PLUGIN_HH

// General stuff
#include <map>
#include <string>
#include <vector>

// Ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <DRC_msgs/PoseStampedArray.h>
#include <DRC_msgs/PoseJointTrajectory.h>

//#include <atlas_msgs/AtlasCommand.h>
//#include <atlas_msgs/AtlasSimInterfaceCommand.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// Gazebo
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

enum MODE_TYPE {
  ON_FEET_MODE,
  ON_NOMINAL_MODE,
  ON_NO_GRAVITY_MODE,
  ON_STAY_DOG_MODE
};

namespace gazebo
{
  /**
   * @class DRCPlugin
   */
  class DRCPlugin : public WorldPlugin
  {
    /// \brief Constructor
  public: DRCPlugin();
    
    /// \brief Destructor
  public: virtual ~DRCPlugin();
    
    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to parent world.
    /// \param[in] _sdf Pointer to sdf element.
  public: void Load( physics::WorldPtr _parent, 
		     sdf::ElementPtr _sdf );
    
    /// \brief Update the controller on every World::Update
  private: void UpdateStates();
    
    
    /// \brief sets robot's absolute world pose
    /// \param[in] _cmd Pose command for the robot
  public: void SetRobotPose(const geometry_msgs::Pose::ConstPtr &_cmd);
    
    /// \brief sets robot's joint positions
    /// \param[in] _cmd configuration made of sensor_msgs::JointState message
    /// \todo: not yet implemented
  public: void SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr
                                       &/*_cmd*/);

    /// \brief Set Joint Animation 
  public: void SetRobotJointAnimation(const trajectory_msgs::JointTrajectory::ConstPtr &_cmd );

    /// \brief Set Pose Animation w.r.t Torso
  public:  void SetRobotPoseAnimation(const DRC_msgs::PoseStampedArray::ConstPtr &_cmd );

    /// \brief Set Robot Joint + Pose Animation w.r.t Torso
  public:  void SetRobotPoseJointAnimation(const DRC_msgs::PoseJointTrajectory::ConstPtr &_cmd );


    /// \brief sets robot mode via ros topic
    /// \sa SetRobotMode(const std::string &_str)
  public: void SetRobotModeTopic(const std_msgs::String::ConstPtr &_str);

    /// \brief sets robot mode
    /// \param[in] _str sets robot mode by a string.  Supported modes are:
    ///  - "no_gravity" Gravity disabled for the robot.
    ///  - "nominal" Nominal "normal" physics.
    ///  - "pinned" Robot is pinned to inertial world by the pelvis.
    ///  - "feet" same as no_gravity except for r_foot and l_foot links.
  public: void SetRobotMode(const std::string &_str);


    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Generic tools for manipulating models                                //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    /// \brief sets robot's absolute world pose
    /// \param[in] _pinLink Link to pin to the world
    /// \param[in] _pose sets the _pinLink world pose before pinning
    /// \param[in] _jp joint and positions for model configuration (TODO)
    /// \param[out] _pinJoint a pin Joint is created
    private: void Teleport(const physics::LinkPtr &_pinLink,
                          physics::JointPtr &_pinJoint,
                          const math::Pose &_pose,
                          const std::map<std::string, double> &/*_jp*/);

    /// \brief sets robot's absolute world pose
    /// \param[in] _pinLink Link to pin to the world
    /// \param[in] _pose sets the _pinLink world pose before pinning
    /// \param[out] _pinJoint a pin Joint is created
    private: void Teleport(const physics::LinkPtr &_pinLink,
                          physics::JointPtr &_pinJoint,
                          const math::Pose &_pose);

    /// \brief add a constraint between 2 links
    /// \param[in] _world a pointer to the current World
    /// \param[in] _model a pointer to the Model the new Joint will be under
    /// \param[in] _link1 parent link in the new Joint
    /// \param[in] _link2 child link in the new Joint
    /// \param[in] _type string specifying joint type
    /// \param[in] _anchor a Vector3 anchor offset of the new joint
    /// \param[in] _axis Vector3 containing xyz axis of the new joint
    /// \param[in] _upper upper linit of the new joint
    /// \param[in] _lower lower linit of the new joint
    /// \return Joint created between _link1 and _link2 under _model.
    private: physics::JointPtr AddJoint(physics::WorldPtr _world,
                                        physics::ModelPtr _model,
                                        physics::LinkPtr _link1,
                                        physics::LinkPtr _link2,
                                        std::string _type,
                                        math::Vector3 _anchor,
                                        math::Vector3 _axis,
                                        double _upper, double _lower,
                                        bool _disableCollision = true);

    /// \brief Remove a joint.
    /// \param[in] _joint Joint to remove.
    private: void RemoveJoint(physics::JointPtr &_joint);

    /// \brief setup Robot ROS publication and sbuscriptions for the Robot
    /// These ros api describes Robot only actions
    private: void LoadRobotROSAPI();

    /// \brief setup ROS publication and sbuscriptions for VRC
    /// These ros api describes interactions between different models
    /// /atlas/cmd_vel - in pinned mode, the robot teleports based on
    ///                      messages from the cmd_vel
    private: void LoadDRCROSAPI();


    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    /// \brief ROS callback queue thread
    private: void ROSQueueThread();
    
    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Drchubo properties and states                                        //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
  private: class Robot {

      /// \brief Load the robot portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
    private: void SetFootParallelToFloor();
    private: void Load( physics::WorldPtr _parent, 
			sdf::ElementPtr _sdf );  
    private: physics::ModelPtr model;
    private: physics::LinkPtr pinLink;
    private: physics::JointPtr pinJoint;
      
      /// \brief keep initial pose of robot to prevent z-drifting when
      /// teleporting the robot.
    private: math::Pose initialPose;
      
      /// \brief flag for successful initialization of atlas
    private: bool isInitialized;
      
      // Helpers
      math::Pose mTempPose;

      /// \brief Mode flags
      int modeType;
    public: double ankleOffset;

      /// \brief drchubo's  Subscribers 
    private: ros::Subscriber subJointAnimation;
    private: ros::Subscriber subPoseAnimation;
    private: ros::Subscriber subPoseJointAnimation;
    private: ros::Subscriber subTrajectory;
    private: ros::Subscriber subPose;
    private: ros::Subscriber subConfiguration;
    private: ros::Subscriber subMode;

      // Store joint pointers
    private: std::vector<physics::JointPtr> mJoints;
      
      friend class DRCPlugin;
    } drchubo;

    // //////////////////////////////////
    // GrabJoint stuff
    private: ros::Subscriber subRobotGrab;
    private: ros::Subscriber subRobotRelease;
    private: physics::JointPtr grabJoint;

    //////////////////////////////////////////
    // DRILL 
    //////////////////////////////////////////
  private: class Drill
    {
      /// \brief set initial configuration of the fire hose link
    private: void SetInitialConfiguration();
      
      /// \brief Load the drc_fire_hose portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
    private: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
      
    private: physics::ModelPtr drillModel;
      // Additional models
      /*
    private: physics::ModelPtr standpipeModel;
    private: physics::ModelPtr valveModel;
    private: physics::JointPtr valveJoint;
      */
      /// joint for pinning a link to the world
    private: physics::JointPtr fixedJoint;
      
      /// joints and links
    private: physics::Joint_V drillJoints;
    private: physics::Link_V drillLinks;
      /// screw joint
    private: physics::JointPtr screwJoint;
    private: double threadPitch;
      
      /// Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
      
    private: physics::LinkPtr couplingLink;
    private: physics::LinkPtr spoutLink;
    private: math::Pose couplingRelativePose;
    private: math::Pose initialDrillPose;
      
      /// \brief flag for successful initialization of fire hose, standpipe
    private: bool isInitialized;
      
      friend class DRCPlugin;
    } drill;  
    

      /// \brief drchubo's Publishers
  private: ros::Publisher jointStatesPub;
  private: ros::Publisher posePub;

    /// Drill callbacks for grabbing or releasing
    /// \brief Cheats to teleport fire hose to hand and make a fixed joint
    /// \param[in] _cmd Relative pose offset between the link and the hand.
  public: void RobotGrabDrill(const geometry_msgs::Pose::ConstPtr &_cmd);
    
    /// \brief remove the fixed joint between robot hand link and fire hose.
    /// \param[in] _cmd not used.
  public: void RobotReleaseLink(const geometry_msgs::Pose::ConstPtr &_cmd);


      /// \brief Are we animating?
  public: bool onJointAnimation;
  public: bool onPoseAnimation;

      /// \brief Animation callbacks
  private:  boost::function<void()> jointAnim_callback;
  private:  boost::function<void()> poseAnim_callback;
  private: void jointAnimation_callback();
  private: void poseAnimation_callback();

      /// \brief Joint and pose current state getters
  private:  void getCurrentJointState();
  private:  void getCurrentPose();

      // Lock
      private: boost::mutex update_mutex;

      ////////////////////////////////////////////////////////////////////////////
      //                                                                        //
      //   Private variables                                                    //
      //                                                                        //
      ////////////////////////////////////////////////////////////////////////////
  private: double lastUpdateTime;
      //private: sensor_msgs::JointState defaultJointState;    
      //private: geometry_msgs::Pose defaultPose;
  private: std::map<std::string, double> defaultJointState_p;
  private: math::Pose defaultPose_p;
    
      /// \brief Pointer to parent world.
  private: physics::WorldPtr world;

      /// \brief Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

      // default ros stuff
  private: ros::NodeHandle* rosNode;
  private: ros::CallbackQueue rosQueue;
  private: boost::thread callbackQueueThread;

      // items below are used for deferred load in case ros is blocking
  private: sdf::ElementPtr sdf;
  private: boost::thread deferredLoadThread;

      // Should be in Robot but let's be practical by now
      static const int mNumJoints;
      static std::string mFullJointNames[];
      static std::string mJointNames[];

  };
/** \} */
/// @}
}
#endif
