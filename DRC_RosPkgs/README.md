Instructions to use DRC package
===============================

1. In DRC_RosPkgs there are 2 ROS packages:
  * DRC_msgs : Customized messages (Array of Stamped poses and pose + joint
    trajectory message)
  * DRC : Utilities to work with DRCHubo using ROS. 
2. DRC depends on DRC_msgs, so you first have to compile DRC_msgs and
   then DRC
3. DRC also uses drchubo.urdf from the package
   drchubo. So make sure you have drchubo on your machine too. 
4. Currently we only support:
   * joint / pose / joint + pose animation
   * Modes no_gravity (for animation) and nominal (gravity)

To compile
-----------

1. Put both DRC and DRC_msgs in a folder in $ROS_PACKAGE_PATH (or
   do export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH in DRC_RosPkgs
2. Build DRC_msgs

	cd DRC_msgs
	mkdir build
	cd build && cmake ..
	make

3. Build DRC. For this, there are 2 things to do before doing make:

   * In DRC/CMakeLists.txt, line 36: Set the drchubo_ROOT_PATH to
     your local folder drchubo.
   * You have to create a link to the huboplus folder needed by Matt's
     code. Follow the instructions from DRC/src/walkPublisher/mattStuff/README.md to do so.
   * Now you cand do the cmake and make procedure
   * Finally, run 
     source DRC/setup.sh in the terminal you will use. This sets GAZEBO_PLUGIN_PATH and LD_LIBRARY_PATH to find our DRCPlugin

4. To run the rubble world example:

   * In one terminal type: roslaunch DRC drchubo_rubble.launch
   * Press Play since we start on Pause Mode
   * Don't be surprised if the robot flies. I will change the node
     to start in an upper position later today
   * In another terminal type: rosrun DRC zmpnode
   * Robot should do the same thing as the video


Example code
--------------

DRC/src/walkPublisher

Topics to which drchubo is subscribed
--------------------------------------

1. Topic to set pose with respect to World (from torso): drchubo/set_pose 
    with type: geometry_msgs::Pose>
  
2. Topic to set joint configuration: drchubo/configuration
   with type: sensor_msgs::JointState>
 
3. Topic to set mode drchubo/mode
   with type std_msgs::String ( so far, 2 modes: no_gravity and nominal )

4. Topic to set pose + joint animation: drchubo/poseJointAnimation
   with type: <DRC_msgs::PoseJointTrajectory>
   USE THIS FOR WALK ANIMATION. 

5. Topic to set only joint animation drchubo/jointAnimation
   with type: trajectory_msgs::JointTrajectory

6. Topic to set only pose animation drchubo/poseAnimation
   with type: DRC_msgs::PoseStampedArray>


ToDo
-----

I hardcoded orientation in poseAnimation to 0,0,0 - for some reason the quaternions are not being read fine
will check that later today
