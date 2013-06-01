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


    /// \brief remove the fixed joint between robot hand link and fire hose.
    /// \param[in] _cmd not used.
    public: void RobotReleaseLink(const geometry_msgs::Pose::ConstPtr &_cmd);


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
                                        bool _disableCollision = false);

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

      /// \brief drchubo's  Subscribers 
    private: ros::Subscriber subJointAnimation;
    private: ros::Subscriber subPoseAnimation;
    private: ros::Subscriber subPoseJointAnimation;
    private: ros::Subscriber subTrajectory;
    private: ros::Subscriber subPose;
    private: ros::Subscriber subConfiguration;
    private: ros::Subscriber subMode;
      
      friend class DRCPlugin;
    } drchubo;


    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Private variables                                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: double lastUpdateTime;


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

  };
/** \} */
/// @}
}
#endif
