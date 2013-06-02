DRC-HUBO
===

Repository for DRC rubble clearing task.

Instructions to use drchubo package
===============================

1. DRC depends on DRC_msgs, so you first have to compile DRC_msgs and
   then drchubo
2. Currently we only support:
   * joint / pose / joint + pose animation
   * Modes no_gravity (for animation) and nominal (gravity)

To compile
-----------

1. Put DRC_msgs in a folder in $ROS_PACKAGE_PATH 
2. Build DRC_msgs

	cd DRC_msgs
	mkdir build
	cd build && cmake ..
	make

3. Build drchubo. For this, there is one thing for you to do before doing make:

   * You have to create a link to the huboplus folder needed by Matt's
     code (which we use for our zmpnode executable for walking). Follow the instructions from apps/zmpnode/mattStuff/README.md to do so.
   * Now you cand do the cmake and make procedure
   * Finally, source the file setup.sh (located in drchubo) in the terminal you will use. This sets GAZEBO_PLUGIN_PATH and LD_LIBRARY_PATH to find our DRCPlugin

4. To run the rubble world example:

   * In one terminal type: roslaunch drchubo drchubo_rubble.launch
   * Press Play since we start on Pause Mode
   * Don't be surprised if the robot flies. I will change the node
     to start in an upper position later today
   * In another terminal type: rosrun drchubo zmpnode
   * Robot should do the same thing as the video


Example code
--------------

drchubo/apps/zmpnode

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


