#! /usr/bin/env python

import roslib; roslib.load_manifest('attila')

from atlas_msgs.msg import WalkDemoAction, \
    WalkDemoActionGoal, \
    WalkDemoGoal, \
    AtlasBehaviorStepData, \
    AtlasBehaviorStepParams, \
    AtlasBehaviorStandParams, \
    AtlasBehaviorManipulateParams

from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
import actionlib
import math
import rospy
import select
import sys
import termios
import tty

# *****************
# AtlasTeleop
# *****************
class AtlasTeleop():


    # ********
    # init
    # ********
    def init(self):
        
        # Creates the SimpleActionClient
        self.client = actionlib.SimpleActionClient('atlas/bdi_control', WalkDemoAction )
        self.mode = rospy.Publisher( '/atlas/mode', String, None, False, True, None )
        self.control_mode = rospy.Publisher( '/atlas/control_mode', String, None, False, True, None )
        
        # Wait until the action server has started up and started listening for goals
        rospy.loginfo("Waiting for atlas/bdi_control")
        self.client.wait_for_server()

    # ********
    # run
    # ********
    def run(self):
        try:
            
            self.init()
            forward = rospy.get_param('Forward')
            lateral = rospy.get_param('Lateral')
            turn = rospy.get_param('Turn')

            self.twist( forward, lateral, turn )

        finally:
            print "Done"
    
    # **********
    # twist     
    # **********
    def twist(self, forward, lateral, turn):

        steps = []
        
        L = rospy.get_param("Forward_Stride_Length")
        L_lat = rospy.get_param("Lateral_Stride_Length")
        R = rospy.get_param("Turn_Radius")
        W = rospy.get_param("Stride_Width")
        X = 0
        Y = 0
        theta = 0
        dTheta = 0
        
        if forward != 0:
            dTheta = turn * 2 * math.asin(L / (2 * (R + rospy.get_param("Stride_Width")/2)))
        else:
            dTheta = turn * rospy.get_param("In_Place_Turn_Size")
        steps = []
        
        # This home step doesn't currently do anything, but it's a 
        # response to bdi not visiting the first step in a trajectory
        home_step = AtlasBehaviorStepData()
        
        # If moving right, first dummy step is on the left
        home_step.foot_index = 1*(lateral < 0)
        home_step.pose.position.y = 0.1
        steps.append(home_step)
        prevX = 0
        prevY = 0
        
        # Builds the sequence of steps needed
        for i in range(rospy.get_param("Walk_Sequence_Length")):
            theta += (turn != 0) * dTheta
            
            # is_right_foot = 1, when stepping with right
            is_even = i%2
            is_odd = 1 - is_even
            is_right_foot = is_even
            is_left_foot = is_odd

            # left = 1, right = -1            
            foot = 1 - 2 * is_right_foot
            
            if turn == 0:
                X = (forward != 0) * (X + forward * L)
                Y = (lateral != 0) * (Y + is_odd * lateral * L_lat) + foot * W / 2
            elif forward != 0:
                # Radius from point to foot (if turning)
                R_foot = R + foot * W/2
                
                # turn > 0 for CCW, turn < 0 for CW
                X = forward * turn * R_foot * math.sin(theta)
                Y = forward * turn * (R - R_foot*math.cos(theta))
                
            elif turn != 0:
                X = turn * W/2 * math.sin(theta)
                Y = turn * W/2 * math.cos(theta)
             
            Q = quaternion_from_euler(0, 0, theta)
            step = AtlasBehaviorStepData()
            
            # One step already exists, so add one to index
            step.step_index = i+1
            
            # Alternate between feet, start with left
            step.foot_index = is_right_foot
            
            #If moving laterally to the left, start with the right foot
            if (lateral > 0):
                step.foot_index = is_left_foot
            
            step.duration = rospy.get_param("Stride_Duration")
            
            step.pose.position.x = X
            step.pose.position.y = Y
            step.pose.position.z = rospy.get_param("Step_Height")
         
            step.pose.orientation.x = Q[0]
            step.pose.orientation.y = Q[1]
            step.pose.orientation.z = Q[2]
            step.pose.orientation.w = Q[3]
            
            step.swing_height = rospy.get_param("Swing_Height")         
            steps.append(step)
        
        # Add final step to bring feet together
        is_right_foot = 1 - steps[-1].foot_index
        is_even = is_right_foot
        # foot = 1 for left, foot = -1 for right
        foot = 1 - 2 * is_right_foot
        
        if turn == 0:
            Y = Y - foot * W
        elif forward != 0:
            
            # R_foot is radius to foot
            R_foot = R + foot * W/2
            #turn > 0 for counter clockwise
            X = forward * turn * R_foot * math.sin(theta)
            Y = forward * turn * (R - R_foot*math.cos(theta))
        else:
            X = turn * W/2 * math.sin(theta)
            Y = turn * W/2 * math.cos(theta)
            
        Q = quaternion_from_euler(0, 0, theta)
        step = AtlasBehaviorStepData()
        step.step_index = len(steps)
        step.foot_index = is_right_foot
        step.duration = rospy.get_param("Stride_Duration")
        step.pose.position.x = X
        step.pose.position.y = Y
        step.pose.position.z = rospy.get_param("Step_Height")
        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]
        step.swing_height = rospy.get_param("Swing_Height")
        
        steps.append(step)

        # 0 for full BDI control, 255 for PID control
        k_effort =  [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
               
        walk_goal = WalkDemoGoal(Header(), WalkDemoGoal.WALK, steps, \
          AtlasBehaviorStepParams(), AtlasBehaviorStandParams(), \
          AtlasBehaviorManipulateParams(),  k_effort )
        
        self.client.send_goal(walk_goal)

            
        self.client.wait_for_result(\
          rospy.Duration(2*rospy.get_param("Stride_Duration") * \
                         len(steps) + 5))


if __name__ == '__main__':
    rospy.init_node('interactive_drive')
    teleop = AtlasTeleop()
    teleop.run()
