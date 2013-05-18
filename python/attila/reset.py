#!/usr/bin/env python

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


rospy.init_node('reset_client')

mode = rospy.Publisher( '/atlas/mode', String, None, False, True, None )
control_mode = rospy.Publisher( '/atlas/control_mode', String, None, False, True, None )

mode.publish("harnessed")
control_mode.publish("Freeze")
control_mode.publish("StandPrep")
rospy.sleep(2.0)
mode.publish("nominal")
rospy.sleep(0.3)
control_mode.publish("Stand")
rospy.sleep(0.3)

if sys.argv[1] == "r":
    proc = subprocess.Popen(['xdotool', 'getactivewindow'], stdout=subprocess.PIPE)
    actwin = proc.stdout.readline()
    os.system("xdotool windowfocus --sync 48234507 && xdotool key \"ctrl+shift+r\" && xdotool windowfocus --sync " + actwin)