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
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_matrix
from nav_msgs.msg import Odometry
import actionlib
import math
import rospy
import select
import sys
import termios
import tty
import numpy
import subprocess

import os
import copy
import json

from gait_statistics import GaitStat

class NineMillionTeleop():
    # Keyboard teleop bindings
    directions = {'u': {"forward":1, "lateral":0, "turn": 1}, \
                  'U': {"forward":1, "lateral":0, "turn": 1}, \
                  'i': {"forward":1, "lateral":0, "turn": 0}, \
                  'I': {"forward":1, "lateral":0, "turn": 0}, \
                  'o': {"forward":1, "lateral":0, "turn":-1}, \
                  'O': {"forward":1, "lateral":0, "turn":-1}, \
                  'j': {"forward":0, "lateral":1, "turn": 0}, \
                  'J': {"forward":0, "lateral":1, "turn": 0}, \
                  'k': {"forward":0, "lateral":0, "turn": 0}, \
                  'K': {"forward":-1, "lateral":0, "turn": 0}, \
                  'l': {"forward":0, "lateral":-1, "turn": 0}, \
                  'L': {"forward":0, "lateral":-1, "turn": 0}, \
                  'm': {"forward":0, "lateral":0, "turn": 0.5}, \
                  ',': {"forward":-0.5, "lateral":0, "turn": 0}, \
                  '.': {"forward":0, "lateral":0, "turn":-0.5}}

    # BDI Controller bindings
    params = {"Forward Stride Length":{ "value":0.15, "min":0, "max":1, \
                                "type":"float"},
              "Lateral Stride Length":{ "value":0.15, "min":0, "max":1, \
                                "type":"float"},
              "Step Height":{"value":0, "min":-1, "max":1, "type":"float"},
              "Stride Duration":{ "value":0.63, "min": 0, "max":100, \
                                "type":"float"},
              "Walk Sequence Length":{"value":20, "min":1, "max":sys.maxint, \
                                "type":"int"},
              "Stride Width":{"value":0.2, "min":0, "max":1, "type":"float"},
              "In Place Turn Size":{"value":math.pi / 16, "min":0, \
                                    "max":math.pi / 2, "type":"float"},
              "Turn Radius":{"value":2, "min":0.01, "max":100, "type":"float"},
              "Swing Height":{"value":0.3, "min":0, "max":1, "type":"float"}}

    # Gait statistics
    stat = GaitStat()

    # For everything that can't be a binding, use if/elif instead
    def process_key(self, ch):
        print "got key %c" % ch
        if self.directions.has_key(ch):
            self.process_movement(ch)
        elif ch == 'r':
            self.reset_to_standing()

        # Log the key and config!
        self.stat.log_config(self.params)
        self.stat.log_command(ch)

        rospy.loginfo("Goal status: " + self.client.get_goal_status_text())
        rospy.loginfo("Goal state: " + actionlib.SimpleGoalState.to_string((self.client.get_state())))
        rospy.loginfo("Canceling all goals")
        self.client.cancel_all_goals()

        if actionlib.SimpleGoalState.to_string((self.client.get_state())) == "ACTIVE":
            rospy.Shutdown("Killing ROS")

    # Select binding values and call twist
    def process_movement(self, ch):
        dir = self.directions[ch]
        self.twist(dir["forward"], dir["lateral"], dir["turn"])

    # initialize
    def __init__(self):
        # Creates the SimpleActionClient
        self.client = actionlib.SimpleActionClient('atlas/bdi_control', WalkDemoAction )
        self.mode = rospy.Publisher( '/atlas/mode', String, None, False, True, None )
        self.control_mode = rospy.Publisher( '/atlas/control_mode', String, None, False, True, None )
        
        # Wait until the action server has started up and started listening for goals
        rospy.loginfo("Waiting for atlas/bdi_control")
        self.client.wait_for_server()

        # Cancel all goals currently running on the server
        rospy.loginfo("Goal status: " + self.client.get_goal_status_text())
        rospy.loginfo("Canceling all goals")
        self.client.cancel_all_goals()

    # Publishes commands to reset robot to a standing position
    def reset_to_standing(self):
        # self.mode.publish("harnessed")
        # self.control_mode.publish("Freeze")
        # self.control_mode.publish("StandPrep")
        # rospy.sleep(2.0)
        # self.mode.publish("nominal")
        # rospy.sleep(0.3)
        # self.control_mode.publish("Stand")
        # rospy.sleep(0.3)
        pass

	# twist
    def twist(self, forward, lateral, turn):

        steps = []
        
        L = self.params["Forward Stride Length"]["value"]
        L_lat = self.params["Lateral Stride Length"]["value"]
        R = self.params["Turn Radius"]["value"]
        W = self.params["Stride Width"]["value"]
        X = 0
        Y = 0
        theta = 0
        dTheta = 0
        
        if forward != 0:
            dTheta = turn * 2 * math.asin(L / (2 * (R + self.params["Stride Width"]["value"]/2)))
        else:
            dTheta = turn * self.params["In Place Turn Size"]["value"]
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
        for i in range(self.params["Walk Sequence Length"]["value"]):
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
            
            step.duration = self.params["Stride Duration"]["value"]
            
            step.pose.position.x = X
            step.pose.position.y = Y
            step.pose.position.z = self.params["Step Height"]["value"]
         
            step.pose.orientation.x = Q[0]
            step.pose.orientation.y = Q[1]
            step.pose.orientation.z = Q[2]
            step.pose.orientation.w = Q[3]
            
            step.swing_height = self.params["Swing Height"]["value"]         
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
        step.duration = self.params["Stride Duration"]["value"]
        step.pose.position.x = X
        step.pose.position.y = Y
        step.pose.position.z = self.params["Step Height"]["value"]
        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]
        step.swing_height = self.params["Swing Height"]["value"]
        
        steps.append(step)

        # 0 for full BDI control, 255 for PID control
        k_effort =  [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 

        walk_goal = WalkDemoGoal(Header(), WalkDemoGoal.WALK, steps, \
          AtlasBehaviorStepParams(), AtlasBehaviorStandParams(), \
          AtlasBehaviorManipulateParams(),  k_effort )
        
        self.client.send_goal(walk_goal)

        self.client.wait_for_result(\
          rospy.Duration(2*self.params["Stride Duration"]["value"] * \
                         len(steps) + 5))

class NineMillionFalls():
    # Brute force status file
    nmf_cfg_file = 'data/nmf_dir/config.txt'
    nmf_dir_file = 'data/nmf_dir/direction.txt'
    nmf_itr_file = 'data/nmf_dir/iteration.txt'
    nmf_stp_file = 'data/nmf_last_step.txt'
    # Search config (starting)
    nmf_config = {"Forward Stride Length":{"value":0.1,  "min":0.1, "max":0.7,  "type":"float", "incr":0.425},
                  "Lateral Stride Length":{"value":0.1,  "min":0.1, "max":0.1,  "type":"float", "incr":0.1},
                  "Stride Duration":      {"value":0.6,  "min":0.6, "max":0.81, "type":"float", "incr":0.01},
                  "Walk Sequence Length": {"value":1,    "min":1,   "max":9,    "type":"int",   "incr":1},
                  "Stride Width":         {"value":0.2,  "min":0.2, "max":0.2,  "type":"float", "incr":0.02},
                  "In Place Turn Size":   {"value":0.1,  "min":0.1, "max":0.1,  "type":"float", "incr":0.1}}

    nmf_direction = {"Forward":   {"key":'i', "conf":["Forward Stride Length", "Stride Duration", "Stride Width", "Walk Sequence Length"]},
                     "Side Left": {"key":'j', "conf":["Lateral Stride Length", "Stride Duration", "Stride Width", "Walk Sequence Length"]},
                     "Side Right":{"key":'l', "conf":["Lateral Stride Length", "Stride Duration", "Stride Width", "Walk Sequence Length"]},
                     "Turn Left": {"key":'m', "conf":["In Place Turn Size", "Stride Duration", "Stride Width", "Walk Sequence Length"]},
                     "Turn Right":{"key":'.', "conf":["In Place Turn Size", "Stride Duration", "Stride Width", "Walk Sequence Length"]}}

    nmf_iteration = {"Iteration": {"value":0, "min":0, "max":1, "incr":1},
                     "Direction": "Forward",
                     "Repeat":0,
                     "Total": 0}


    def __init__(self):
        # Prepare file paths
        __location__ = os.path.realpath( os.path.dirname(__file__) )
        __location__ = os.path.dirname( __location__ )
        self.nmf_cfg_file = os.path.join( __location__ , self.nmf_cfg_file )
        self.nmf_dir_file = os.path.join( __location__ , self.nmf_dir_file )
        self.nmf_itr_file = os.path.join( __location__ , self.nmf_itr_file )
        self.nmf_stp_file = os.path.join( __location__ , self.nmf_stp_file )
        print "nmf_cfg_file: %s" % self.nmf_cfg_file
        print "nmf_dir_file: %s" % self.nmf_dir_file
        print "nmf_itr_file: %s" % self.nmf_itr_file
        print "nmf_stp_file: %s" % self.nmf_stp_file

        # Double up (starting)
        c1 = copy.deepcopy(self.nmf_config)
        c2 = copy.deepcopy(self.nmf_config)
        self.nmf_config = [c1, c1]
        d1 = copy.deepcopy(self.nmf_direction)
        d2 = copy.deepcopy(self.nmf_direction)
        self.nmf_direction = [d1, d1]
        i1 = copy.deepcopy(self.nmf_iteration)
        i2 = copy.deepcopy(self.nmf_iteration)
        self.nmf_iteration = [i1, i1]
        # self.nmf_config = [self.nmf_config]
        # self.nmf_direction = [self.nmf_direction]
        # self.nmf_iteration = [self.nmf_iteration]

    def next_config(self, cfg, dire, itr):
        for key in cfg.keys():
            if key in dire[itr["Direction"]]["conf"]:
                if cfg[key]["incr"] == 0:
                    pass
                elif cfg[key]["value"] + cfg[key]["incr"] > cfg[key]["max"]:
                    cfg[key]["value"] = cfg[key]["min"]
                else:
                    cfg[key]["value"] += cfg[key]["incr"]
                    return False
        return True

    def next_dir(self, dire, itr):
        key_ind = -1;
        num_keys = len(dire)
        for i in range(num_keys):
            if dire.keys()[i] == itr["Direction"]:
                key_ind = i
                if key_ind < num_keys - 1:
                    itr["Direction"] = dire.keys()[key_ind+1]
                    return False
                else:
                    itr["Direction"] = dire.keys()[0]
                    return True

    def next_itr(self, itr):
        itr["Total"] += 1
        itr["Iteration"]["value"] += itr["Iteration"]["incr"]
        if itr["Iteration"]["value"] < itr["Iteration"]["max"]:
            return False
        else:
            itr["Iteration"]["value"] = itr["Iteration"]["min"]
            return True

    def __next(self, cfg, dire, itr):
        if self.next_config(cfg, dire, itr):
            if self.next_dir(dire, itr):
                return True
        return False

    def next_fun(self):
        # Increment iterations over all
        # save last one for updating other parameters
        for i in range(len(self.nmf_config)-1):
            self.next_itr(self.nmf_iteration[i])
        # Update other parameters
        last = len(self.nmf_config)-1
        if self.next_itr(self.nmf_iteration[last]):
            i = 0
            while self.__next(self.nmf_config[i],
                              self.nmf_direction[i],
                              self.nmf_iteration[i]):
                i += 1
                if i == len(self.nmf_config):
                    return False
        return True

    def fall(self):
        rospy.sleep(3.0)
        data = rospy.client.wait_for_message("/atlas/imu", Imu)
        Q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        M = quaternion_matrix(Q)
        print M[2][2]
        if M[2][2] < 0.5:
            print M[2][2]
            return True
        return False

    def teleop_config(self, nmt, cfg):
        for key in cfg.keys():
            nmt.params[key]["value"] = cfg[key]["value"]

    def check_ground_truth(self, ob, oa):
        b = [ob.pose.pose.position.x, ob.pose.pose.position.y, ob.pose.pose.position.z]
        a = [oa.pose.pose.position.x, oa.pose.pose.position.y, oa.pose.pose.position.z]

        aa = numpy.array(a)
        bb = numpy.array(b)
        dist = numpy.linalg.norm((aa-bb), ord=1)

        return dist < 0.01

    def xdo_reset_pose(self):
        # proc = subprocess.Popen(['xdotool', 'getactivewindow'], stdout=subprocess.PIPE)
        # actwin = proc.stdout.readline()
        # os.system("xdotool windowfocus --sync 62914571 && xdotool key \"ctrl+shift+r\" && xdotool windowfocus --sync " + actwin)
        pass

    def manual_reset_atlas(self):
        nmt = NineMillionTeleop()
        nmt.reset_to_standing()
        self.xdo_reset_pose()

    def teleop(self, nmt):
        nmt.reset_to_standing()
        self.xdo_reset_pose()
        for i in range(len(self.nmf_config)):
            for j in range(self.nmf_iteration[i]["Repeat"]):
                # store pose
                odm_before = rospy.client.wait_for_message("/ground_truth_odom", Odometry)

                self.teleop_config(nmt, self.nmf_config[i])
                nmt.process_key(self.nmf_direction[i][self.nmf_iteration[i]["Direction"]]["key"])

                # check if pose is different
                # if not, wait forever...?
                odm_after = rospy.client.wait_for_message("/ground_truth_odom", Odometry)

                if self.check_ground_truth(odm_before, odm_after):
                    # we failed to move, restart ourselves!
                    rospy.loginfo("Failed to detect motion, restarting")
                    rospy.signal_shutdown("Shutdown")

                if self.fall():
                    nmt.process_key('r')
                    self.xdo_reset_pose()
                    return
                # else:
                #     nmt.process_key('!') # <-- I f'd up

    def run(self):
        # main loop
        while True:
            self.save() # 
            print "new 9 mil teleop"
            nmt = NineMillionTeleop()
            self.teleop(nmt)
            self.load() # In case the user wants to change something
            if not self.next_fun():
                break

    def save(self):
        rospy.loginfo("[NMF] Saving configurations")
        with open(self.nmf_cfg_file, 'w') as nmf_obj:
            json.dump(self.nmf_config, nmf_obj, indent=4, separators=(',',': '))
        with open(self.nmf_dir_file, 'w') as nmf_obj:
            json.dump(self.nmf_direction, nmf_obj, indent=4, separators=(',',': '))
        with open(self.nmf_itr_file, 'w') as nmf_obj:
            json.dump(self.nmf_iteration, nmf_obj, indent=4, separators=(',',': '))
        with open(self.nmf_stp_file, 'w') as nmf_obj:
            nmf_obj.write(self.nmf_iteration[-1]["Direction"])

    def load(self):
        rospy.loginfo("[NMF] Loading configurations")
        with open(self.nmf_cfg_file, 'r') as nmf_obj:
            self.nmf_config = json.load(nmf_obj)
        with open(self.nmf_dir_file, 'r') as nmf_obj:
            self.nmf_direction = json.load(nmf_obj)
        with open(self.nmf_itr_file, 'r') as nmf_obj:
            self.nmf_iteration = json.load(nmf_obj)

    def info(self):
        print json.dumps(self.nmf_config, indent=4, separators=(',',': '))
        # print json.dumps(self.nmf_direction, indent=4, separators=(',',': '))
        print json.dumps(self.nmf_iteration, indent=4, separators=(',',': '))

if __name__ == '__main__':

    if len(sys.argv) == 1:
        rospy.init_node('walking_client')
        nmf = NineMillionFalls()
        nmf.load()
        nmf.run()
    elif sys.argv[1] == "save":
        nmf = NineMillionFalls()
        nmf.save()
    elif sys.argv[1] == "dry":
        nmf = NineMillionFalls()
        nmf.load()
        while nmf.next_fun():
            pass
        print nmf.nmf_iteration
    elif sys.argv[1] == "help":
        print "usage: roslaunch attila nine_million_falls.launch"
        print "usage: python nine_million_falls.py [save|dry]"
    else:
        rospy.init_node('walking_client')
        nmf = NineMillionFalls()
        nmf.load()
        nmf.run()
