#!/usr/bin/env python
import roslib; roslib.load_manifest('attila')

from tf.transformations import quaternion_from_euler

import rospy
from sensor_msgs.msg import Imu
import time
import re

from tf.transformations import quaternion_matrix

from nav_msgs.msg import Odometry

from std_msgs.msg import Empty

# def callback(data):
#   nice_data = str(data)
#   start = 'y: ' #this gets the FIRST occurance of x and y, which is the orientation.
#   end = ' z:'
#   result = ((nice_data.split(start))[1].split(end)[0])
#   print result
#   if abs(float(result)) > 0.45: #I observed that the y coordinate goes above .45 when he falls
#   	print "I\'ve fallen down and I can\'t get up!"
#   	#pro tip: the rostopic will buffer, so don't use time.sleep() for delays because then you'll get way out of sync

# def listener():
#   rospy.init_node('watcher')
#   rospy.Subscriber("/atlas/imu", Imu, callback)
#   # spin() simply keeps python from exiting until this node is stopped
#   rospy.spin()

# listener()

rospy.init_node('watcher')
# data = rospy.client.wait_for_message("/atlas/imu", Imu)

# print data.orientation

# Q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

# print quaternion_matrix(Q)

data = rospy.client.wait_for_message("/ground_truth_odom", Odometry)
print data.pose.pose

Q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]

print "xform:"
print quaternion_matrix(Q)

# pub = rospy.Publisher('/gazebo/reset_models', Empty)
# pub.publish(Empty)

# start = 'y: ' #this gets the FIRST occurance of x and y, which is the orientation.
# end = ' z:'
# result = ((data.split(start))[1].split(end)[0])
# print result

# import subprocess

# proc = subprocess.Popen(['xdotool', 'getactivewindow'], stdout=subprocess.PIPE)

# print proc.stdout.readline()



